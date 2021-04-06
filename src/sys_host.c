/*
 * This file is part of mod-system-control.
 */

#include "sys_host.h"
#include "sys_host_impl.h"

#include <pthread.h>

static volatile bool sys_host_thread_running = false;
static int sys_host_shmfd;
static sys_serial_shm_data* sys_host_data;
static pthread_t sys_host_thread;
static bool s_debug;

static void* sys_host_thread_run(void* const arg)
{
    while (sys_host_thread_running)
    {
        printf("got sys host event %i\n", sys_host_data->buffer[0]);

        sem_wait(&sys_host_data->sem);
    }

    return NULL;

    // unused
    (void)arg;
}

void sys_host_setup(const bool debug)
{
    s_debug = debug;

    if (! sys_serial_open(&sys_host_shmfd, &sys_host_data, true))
    {
        fprintf(stderr, "sys_host shared memory failed\n");
        return;
    }

    sys_host_thread_running = true;
    pthread_create(&sys_host_thread, NULL, sys_host_thread_run, NULL);
}

void sys_host_destroy()
{
    if (sys_host_data == NULL)
        return;

    sys_host_thread_running = false;
    sem_post(&sys_host_data->sem);
    pthread_join(sys_host_thread, NULL);

    sys_serial_close(sys_host_shmfd, sys_host_data, true);
}
