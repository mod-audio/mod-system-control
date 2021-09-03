/*
 * This file is part of mod-system-control.
 */

#include "sys_host.h"
#include "cli.h"
#include "serial_rw.h"

#include "../mod-controller-proto/mod-protocol.h"

#define SERVER_MODE
#include "sys_host_impl.h"

#include <pthread.h>
#include <stdlib.h>

static volatile bool sys_host_thread_running = false;
static volatile bool sys_host_values_changed = false;
static int sys_host_shmfd;
static sys_serial_shm_data* sys_host_data;
static pthread_t sys_host_thread;
static int sys_host_has_msgs;
static bool s_debug;

// compressor state
static int compressor_mode = 0;
static float compressor_release = 100.0f;
static float pedalboard_gain = 0.0f;

// noise gate state
static int noisegate_channel = 0;
static float noisegate_decay = 10.0f;
static float noisegate_threshold = -60.0f;

#if defined(_MOD_DEVICE_DWARF)
#define HMI_NUM_PAGES 8
#define HMI_NUM_SUBPAGES 3
#define HMI_NUM_ACTUATORS 6
#endif

// fallback
#ifndef HMI_NUM_PAGES
#define HMI_NUM_PAGES 1
#define HMI_NUM_SUBPAGES 1
#define HMI_NUM_ACTUATORS 2
#endif

// page cache handling
typedef struct HMI_CACHE_T {
    // values match mod-host message size
    char led[32];
    char label[24];
    char value[24];
    char unit[24];
    char indicator[32];
} hmi_cache_t;
static hmi_cache_t* hmi_cache[HMI_NUM_PAGES * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS];
static int hmi_page = 0;
static int hmi_subpage = 0;
static int hmi_page_or_subpage_changed = 0;
static bool hmi_io_values_requested = false;

static bool read_host_values(void)
{
    char buf[0xff];
    if (! read_file(buf, "/data/audioproc.txt", s_debug))
        return false;

    int cmode = -1, ngchannel = -1;
    float crelease = 0, pgain = 0, ngdecay = 0, ngthreshold = 0;
    sscanf(buf, "%i\n%f\n%f\n%i\n%f\n%f\n",
           &cmode, &crelease, &pgain, &ngchannel, &ngdecay, &ngthreshold);

    // bail out if any value is invalid
    if (cmode < 0 || cmode > 4)
        return false;
    if (crelease < 50.0f || crelease > 500.0f)
        return false;
    if (pgain < -30.0f || pgain > 20.0f)
        return false;
    if (ngchannel < 0 || ngchannel > 3)
        return false;
    if (ngdecay < 1.0f || ngdecay > 500.0f)
        return false;
    if (ngthreshold < -70.0f || ngthreshold > -10.0f)
        return false;

    if (s_debug) {
        printf("%s success, values: %i, %f, %f, %i, %f, %f\n",
               __func__, cmode, crelease, pgain, ngchannel, ngdecay, ngthreshold);
    }

    // all good!
    compressor_mode = cmode;
    compressor_release = crelease;
    pedalboard_gain = pgain;
    noisegate_channel = ngchannel;
    noisegate_decay = ngdecay;
    noisegate_threshold = ngthreshold;
    return true;
}

static void write_host_values(void)
{
    const int cmode = compressor_mode;
    const float crelease = compressor_release;
    const float pgain = pedalboard_gain;
    const int ngchannel = noisegate_channel;
    const float ngdecay = noisegate_decay;
    const float ngthreshold = noisegate_threshold;

    char buf[0xff];
    snprintf(buf, sizeof(buf), "%i\n%f\n%f\n%i\n%f\n%f\n",
             cmode, crelease, pgain, ngchannel, ngdecay, ngthreshold);
    buf[sizeof(buf)-1] = '\0';
    write_file(buf, "/data/audioproc.txt", s_debug);
}

static void* sys_host_thread_run(void* const arg)
{
    while (sys_host_thread_running)
    {
        if (sys_host_values_changed)
        {
            write_host_values();
            sys_host_values_changed = false;
        }

        if (sem_timedwait_secs(&sys_host_data->server.sem, 5))
            continue;

        if (! sys_host_thread_running)
            break;

        sys_host_has_msgs = 1;
    }

    return NULL;

    // unused
    (void)arg;
}

static bool hmi_command_cache_add(const uint8_t page,
                                  uint8_t subpage,
                                  const sys_serial_event_type etype,
                                  char msg[SYS_SERIAL_SHM_DATA_SIZE])
{
    if (page >= HMI_NUM_PAGES)
        return false;
    if (subpage >= HMI_NUM_SUBPAGES)
        return false;

    char actuator[8];
    memset(actuator, 0, sizeof(actuator));
    for (uint8_t i=0; i<sizeof(actuator) && msg[i] != '\0'; ++i)
        if ((actuator[i] = msg[i]) == ' ')
            actuator[i] = '\0';

    // sanity check
    for (uint8_t i=0; i < sizeof(actuator) && actuator[i] != '\0'; ++i)
        if (actuator[i] < '0' || actuator[i] > '9')
            return false;
    if (actuator[sizeof(actuator)-1] != '\0')
        return false;

    const int actuatorId = atoi(actuator);

    if (actuatorId >= HMI_NUM_ACTUATORS)
        return false;

    bool matching_subpage;
#ifdef _MOD_DEVICE_DWARF
    // special exception for dwarf
    if (actuatorId >= 3)
    {
        subpage = 0;
        matching_subpage = true;
    }
    else
#endif
    {
        matching_subpage = subpage == hmi_subpage;
    }

    const size_t index = hmi_page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + actuatorId;
    hmi_cache_t* cache = hmi_cache[index];

    if (cache == NULL)
    {
        hmi_cache[index] = cache = calloc(1, sizeof(hmi_cache_t));

        if (cache == NULL)
            return false;
    }

    const bool ret = page == hmi_page && matching_subpage;

    switch (etype)
    {
    case sys_serial_event_type_led:
        strncpy(cache->led, msg, sizeof(cache->led)-1);
        break;
    case sys_serial_event_type_name:
        strncpy(cache->label, msg, sizeof(cache->label)-1);
        break;
    case sys_serial_event_type_unit:
        strncpy(cache->unit, msg, sizeof(cache->unit)-1);
        break;
    case sys_serial_event_type_value:
        strncpy(cache->value, msg, sizeof(cache->value)-1);
        break;
    case sys_serial_event_type_widget_indicator:
        strncpy(cache->indicator, msg, sizeof(cache->indicator)-1);
        break;
    default:
        break;
    }

    if (s_debug && !ret) {
        printf("%s cached: %u, %u, '%s'\n", __func__, page, subpage, msg);
    }

    return ret;
}

static void hmi_command_cache_remove(const uint8_t page, uint8_t subpage, char msg[SYS_SERIAL_SHM_DATA_SIZE])
{
    if (s_debug) {
        printf("%s called with values: %u, %u, '%s'\n", __func__, page, subpage, msg);
    }
    if (page >= HMI_NUM_PAGES)
        return;
    if (subpage >= HMI_NUM_SUBPAGES)
        return;

    char actuator[8];
    memset(actuator, 0, sizeof(actuator));
    for (uint8_t i=0; i<sizeof(actuator) && msg[i] != '\0'; ++i)
        if ((actuator[i] = msg[i]) == ' ')
            actuator[i] = '\0';

    // sanity check
    for (uint8_t i=0; i < sizeof(actuator) && actuator[i] != '\0'; ++i)
        if (actuator[i] < '0' || actuator[i] > '9')
            return;
    if (actuator[sizeof(actuator)-1] != '\0')
        return;

    const int actuatorId = atoi(actuator);

    if (actuatorId >= HMI_NUM_ACTUATORS)
        return;

#ifdef _MOD_DEVICE_DWARF
    // special exception for dwarf
    if (actuatorId >= 3)
        subpage = 0;
#endif

    const size_t index = hmi_page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + actuatorId;

    if (s_debug) {
        printf("%s has index %lu and cache pointer %p\n", __func__, index, hmi_cache[index]);
    }

    if (hmi_cache[index] != NULL)
    {
        free(hmi_cache[index]);
        hmi_cache[index] = NULL;
    }
}

static void send_command_to_hmi(struct sp_port* const serialport, const char* const sys_cmd,
                                char msg[SYS_SERIAL_SHM_DATA_SIZE], const bool quoted)
{
    const uint8_t len = strlen(msg);
    const size_t msg_offset = _CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2 /* spaces */ + (quoted ? 1 : 0);

    // move value string
    memmove(msg + msg_offset, msg, len+1);

    // place command at start of msg
    memcpy(msg, sys_cmd, _CMD_SYS_LENGTH);

    // store msg size
    snprintf(msg + (_CMD_SYS_LENGTH + 1), _CMD_SYS_DATA_LENGTH + 1, "%02x", len);

    // spaces
    msg[_CMD_SYS_LENGTH] = ' ';
    msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 1] = ' ';

    // quotes, if needed
    if (quoted)
    {
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2] = msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 3];
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 3] = ' ';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 4] = '"';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + len + 3] = '"';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + len + 4] = '\0';
    }

    if (s_debug)
    {
        fprintf(stdout, "send_command_to_hmi '%s'\n", msg);
        fflush(stdout);
    }

    // write message
    write_or_close(serialport, msg);

    // response
    serial_read_ignore_until_zero(serialport);
}

static void send_command_to_host(const sys_serial_event_type etype, const char* const value)
{
    if (sys_host_data == NULL)
        return;

    sys_serial_shm_data_channel* const data = &sys_host_data->client;

    if (! sys_serial_write(data, etype, value))
        return;

    sem_post(&data->sem);
}

static void send_command_to_host_int(const sys_serial_event_type etype, const int value)
{
    char str[24];
    snprintf(str, sizeof(str), "%i", value);
    str[sizeof(str)-1] = '\0';
    send_command_to_host(etype, str);
}

static void send_command_to_host_float(const sys_serial_event_type etype, const float value)
{
    char str[32];
    snprintf(str, sizeof(str), "%f", value);
    str[sizeof(str)-1] = '\0';
    send_command_to_host(etype, str);
}

static void sys_host_resend_hmi(struct sp_port* const serialport)
{
    size_t index;
    hmi_cache_t* cache;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];
    int subpage;

    for (int i=0; i<HMI_NUM_ACTUATORS; ++i)
    {
#ifdef _MOD_DEVICE_DWARF
        // special exception for dwarf
        if (i >= 3)
            subpage = 0;
        else
#endif
            subpage = hmi_subpage;

        index = hmi_page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + i;
        cache = hmi_cache[index];

        if (cache == NULL)
            continue;

        if (cache->led[0] != '\0')
        {
            memcpy(msg, cache->led, sizeof(cache->led));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED, msg, false);
        }
        if (cache->label[0] != '\0')
        {
            memcpy(msg, cache->label, sizeof(cache->label));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_NAME, msg, true);
        }
        if (cache->unit[0] != '\0')
        {
            memcpy(msg, cache->unit, sizeof(cache->unit));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_UNIT, msg, true);
        }
        if (cache->value[0] != '\0')
        {
            memcpy(msg, cache->value, sizeof(cache->value));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_VALUE, msg, true);
        }
        if (cache->indicator[0] != '\0')
        {
            memcpy(msg, cache->indicator, sizeof(cache->indicator));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_WIDGET_INDICATOR, msg, false);
        }
    }
}

void sys_host_setup(const bool debug)
{
    s_debug = debug;

    if (! sys_serial_open(&sys_host_shmfd, &sys_host_data))
    {
        fprintf(stderr, "sys_host shared memory failed\n");
        return;
    }

    read_host_values();
    sys_host_thread_running = true;
    pthread_create(&sys_host_thread, NULL, sys_host_thread_run, NULL);
}

void sys_host_process(struct sp_port* const serialport)
{
    if (sys_host_data == NULL)
        return;

    if (hmi_page_or_subpage_changed)
    {
        // NOTE this is racy, workaround for mod-ui side handling messages slower than us
        if (++hmi_page_or_subpage_changed == 10)
        {
            hmi_page_or_subpage_changed = 0;
            sys_host_resend_hmi(serialport);
        }
    }

    if (hmi_io_values_requested)
    {
        hmi_io_values_requested = false;
        send_command_to_host_int(sys_serial_event_type_compressor_mode, compressor_mode);
        send_command_to_host_float(sys_serial_event_type_compressor_release, compressor_release);
        send_command_to_host_int(sys_serial_event_type_noisegate_channel, noisegate_channel);
        send_command_to_host_float(sys_serial_event_type_noisegate_decay, noisegate_decay);
        send_command_to_host_float(sys_serial_event_type_noisegate_threshold, noisegate_threshold);
        send_command_to_host_float(sys_serial_event_type_pedalboard_gain, pedalboard_gain);
    }

    if (! __sync_bool_compare_and_swap(&sys_host_has_msgs, 1, 0))
        return;

    sys_serial_shm_data_channel* const data = &sys_host_data->server;

    sys_serial_event_type etype;
    uint8_t page, subpage;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];

    while (data->head != data->tail)
    {
        if (! sys_serial_read(data, &etype, &page, &subpage, msg))
            continue;

        switch (etype)
        {
        case sys_serial_event_type_special_req:
            if (strcmp(msg, "restart") == 0)
            {
                hmi_io_values_requested = true;
                hmi_page = 0;
                hmi_subpage = 0;
            }
            else if (strcmp(msg, "pages") == 0)
            {
                hmi_page = page;
                hmi_subpage = subpage;
            }
            break;
        case sys_serial_event_type_unassign:
            hmi_command_cache_remove(page, subpage, msg);
            break;
        case sys_serial_event_type_led:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED, msg, false);
            break;
        case sys_serial_event_type_name:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_NAME, msg, true);
            break;
        case sys_serial_event_type_unit:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_UNIT, msg, true);
            break;
        case sys_serial_event_type_value:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_VALUE, msg, true);
            break;
        case sys_serial_event_type_widget_indicator:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_WIDGET_INDICATOR, msg, false);
            break;
        default:
            break;
        }
    }
}

void sys_host_destroy(void)
{
    if (sys_host_data == NULL)
        return;

    sys_host_thread_running = false;
    sem_post(&sys_host_data->server.sem);
    pthread_join(sys_host_thread, NULL);

    sys_serial_close(sys_host_shmfd, sys_host_data);
}

int sys_host_get_compressor_mode(void)
{
    return compressor_mode;
}

float sys_host_get_compressor_release(void)
{
    return compressor_release;
}

int sys_host_get_noisegate_channel(void)
{
    return noisegate_channel;
}

float sys_host_get_noisegate_decay(void)
{
    return noisegate_decay;
}

float sys_host_get_noisegate_threshold(void)
{
    return noisegate_threshold;
}

float sys_host_get_pedalboard_gain(void)
{
    return pedalboard_gain;
}

void sys_host_set_compressor_mode(const int mode)
{
    compressor_mode = mode;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_compressor_mode, mode);
}

void sys_host_set_compressor_release(const float value)
{
    compressor_release = value;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_compressor_release, value);
}

void sys_host_set_noisegate_channel(const int channel)
{
    noisegate_channel = channel;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_noisegate_channel, channel);
}

void sys_host_set_noisegate_decay(const float value)
{
    noisegate_decay = value;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_noisegate_decay, value);
}

void sys_host_set_noisegate_threshold(const float value)
{
    noisegate_threshold = value;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_noisegate_threshold, value);
}

void sys_host_set_pedalboard_gain(const float value)
{
    pedalboard_gain = value;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_pedalboard_gain, value);
}

void sys_host_set_hmi_page(const int page)
{
    if (hmi_page == page)
        return;

    hmi_page = page;
    hmi_page_or_subpage_changed = 1;
}

void sys_host_set_hmi_subpage(const int subpage)
{
    if (hmi_subpage == subpage)
        return;

    hmi_subpage = subpage;
    hmi_page_or_subpage_changed = 1;
}
