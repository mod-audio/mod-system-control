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

//tmp, remove later
static int num_hex_digits(unsigned n)
{
    if (!n) return 1;

    int ret = 0;
    for (; n; n >>= 4) {
        ++ret;
    }
    return ret;
}

static uint32_t int_to_hex_str(int32_t num, char *string)
{
    const char hex_lookup[] = "0123456789abcdef";
    int len = num_hex_digits(num);

    if (len & 1) {
        *string++ = '0';
    }
    string[len] = '\0';

    for (--len; len >= 0; num >>= 4, --len) {
        string[len] = hex_lookup[num & 0xf];
    }

    return 0;
}

static void* sys_host_thread_run(void* const arg)
{
    while (sys_host_thread_running)
    {
        sem_wait(&sys_host_data->sem);

        if (! sys_host_thread_running)
            break;

        sys_host_has_msgs = 1;
    }

    return NULL;

    // unused
    (void)arg;
}

static void sys_host_send_command(struct sp_port* const serialport, const char *command, const char *arguments)
{
    char buffer[30];
    memset(buffer, 0, sizeof buffer);

    //copy command
    uint8_t i = 0;
    while (*command && (*command != '%' && *command != '.')) {
        buffer[i++] = *command;
        command++;
    }

    if (arguments) {
        //add size as hex number
        char str_bfr[9] = {};
        i+=2;
        i += int_to_hex_str(strlen(arguments), str_bfr);
        strcat(buffer, str_bfr);

        buffer[i++] = ' ';

        strcat(buffer, arguments);
    }
    else
        buffer[_CMD_SYS_LENGTH] = '\0';

    write_or_close(serialport, buffer);
    serial_read_ignore_until_zero(serialport);
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

    sys_serial_event_type etype;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];

    while (sys_host_data->head != sys_host_data->tail)
    {
        if (! sys_serial_read(sys_host_data, &etype, msg))
            continue;

        switch (etype)
        {
        case sys_serial_event_type_led: {
            sys_host_send_command(serialport, CMD_SYS_CHANGE_LED, &msg[3]);
            break;
        }
        case sys_serial_event_type_name: {
            char buffer[20];
            memset(buffer, 0, sizeof buffer);
            strncpy (buffer, &msg[3], 2);
            strcat(buffer, "\"");
            strcat(buffer, &msg[5]);
            strcat(buffer, "\"");
            printf("name change %02x '%s'\n", etype, buffer);
            sys_host_send_command(serialport, CMD_SYS_CHANGE_NAME, buffer);
            break;
        }
        case sys_serial_event_type_unit: {
            char buffer[20];
            memset(buffer, 0, sizeof buffer);
            strncpy (buffer, &msg[3], 2);
            strcat(buffer, "\"");
            strcat(buffer, &msg[5]);
            strcat(buffer, "\"");
            printf("unit change %02x '%s'\n", etype, buffer);
            sys_host_send_command(serialport, CMD_SYS_CHANGE_UNIT, buffer);
            break;
        }
        case sys_serial_event_type_value: {
            char buffer[20];
            memset(buffer, 0, sizeof buffer);
            strncpy (buffer, &msg[3], 2);
            strcat(buffer, "\"");
            strcat(buffer, &msg[5]);
            strcat(buffer, "\"");
            printf("value change %02x '%s'\n", etype, buffer);
            sys_host_send_command(serialport, CMD_SYS_CHANGE_VALUE, buffer);
            break;
        }
        case sys_serial_event_type_widget_indicator: {
            sys_host_send_command(serialport, CMD_SYS_CHANGE_WIDGET_INDICATOR, &msg[3]);
            break;
        }
        default:
            break;
        }

        // TODO do something with this value
        printf("got sys host event %02x '%s'\n", etype, &msg[3]);
    }
}

void sys_host_destroy()
{
    if (sys_host_data == NULL)
        return;

    sys_host_thread_running = false;
    sem_post(&sys_host_data->sem);
    pthread_join(sys_host_thread, NULL);

    sys_serial_close(sys_host_shmfd, sys_host_data);
}
