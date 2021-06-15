/*
 * This file is part of mod-system-control.
 */

#include "sys_host.h"
#include "serial_rw.h"

#include "../mod-controller-proto/mod-protocol.h"

#define SERVER_MODE
#include "sys_host_impl.h"

#include <pthread.h>
#include <stdlib.h>

static volatile bool sys_host_thread_running = false;
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
static float noisegate_threshold = -60.0f;
static float noisegate_decay = 10.0f;

static void* sys_host_thread_run(void* const arg)
{
    while (sys_host_thread_running)
    {
        sem_wait(&sys_host_data->server.sem);

        if (! sys_host_thread_running)
            break;

        sys_host_has_msgs = 1;
    }

    return NULL;

    // unused
    (void)arg;
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

void sys_host_setup(const bool debug)
{
    s_debug = debug;

    if (! sys_serial_open(&sys_host_shmfd, &sys_host_data))
    {
        fprintf(stderr, "sys_host shared memory failed\n");
        return;
    }

    sys_host_thread_running = true;
    pthread_create(&sys_host_thread, NULL, sys_host_thread_run, NULL);
}

void sys_host_process(struct sp_port* const serialport)
{
    if (sys_host_data == NULL)
        return;
    if (! __sync_bool_compare_and_swap(&sys_host_has_msgs, 1, 0))
        return;

    sys_serial_shm_data_channel* const data = &sys_host_data->server;

    sys_serial_event_type etype;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];

    while (data->head != data->tail)
    {
        if (! sys_serial_read(data, &etype, msg))
            continue;

        switch (etype)
        {
        case sys_serial_event_type_null:
            break;
        case sys_serial_event_type_led:
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED, msg, false);
            break;
        case sys_serial_event_type_name:
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_NAME, msg, true);
            break;
        case sys_serial_event_type_unit:
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_UNIT, msg, true);
            break;
        case sys_serial_event_type_value:
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_VALUE, msg, true);
            break;
        case sys_serial_event_type_widget_indicator:
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

float sys_host_get_noisegate_threshold(void)
{
    return noisegate_threshold;
}

float sys_host_get_noisegate_decay(void)
{
    return noisegate_decay;
}

float sys_host_get_pedalboard_gain(void)
{
    return pedalboard_gain;
}

void sys_host_set_compressor_mode(const int mode)
{
    compressor_mode = mode;
    send_command_to_host_int(sys_serial_event_type_compressor_mode, mode);
}

void sys_host_set_compressor_release(const float value)
{
    compressor_release = value;
    send_command_to_host_float(sys_serial_event_type_compressor_release, value);
}

void sys_host_set_noisegate_channel(const int channel)
{
    noisegate_channel = channel;
    send_command_to_host_int(sys_serial_event_type_noisegate_channel, channel);
}

void sys_host_set_noisegate_threshold(const float value)
{
    noisegate_threshold = value;
    send_command_to_host_float(sys_serial_event_type_noisegate_threshold, value);
}

void sys_host_set_noisegate_decay(const float value)
{
    noisegate_decay = value;
    send_command_to_host_float(sys_serial_event_type_noisegate_decay, value);
}

void sys_host_set_pedalboard_gain(const float value)
{
    pedalboard_gain = value;
    send_command_to_host_float(sys_serial_event_type_pedalboard_gain, value);
}
