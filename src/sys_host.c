/*
 * This file is part of mod-system-control.
 */

#include "sys_host.h"
#include "serial_rw.h"

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

static void* sys_host_thread_run(void* const arg)
{
    /*
    sys_serial_event_type etype;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];
    */

    while (sys_host_thread_running)
    {
        sem_wait(&sys_host_data->sem);

        if (! sys_host_thread_running)
            break;

        sys_host_has_msgs = 1;

        /*
        while (sys_host_data->head != sys_host_data->tail)
        {
            if (! sys_serial_read(sys_host_data, &etype, msg))
                continue;

            // TODO do something with this value
            printf("got sys host event %02x '%s'\n", etype, msg);
        }
        */
    }

    return NULL;

    // unused
    (void)arg;
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
            // TODO hardcoded
            write_or_close(serialport, !strcmp(msg, "green") ? "ls 0 3 0 255 0" : "ls 0 3 255 0 0");
            serial_read_ignore_until_zero(serialport);
            break;
        }
        default:
            break;
        }

        // TODO do something with this value
        printf("got sys host event %02x '%s'\n", etype, msg);
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
