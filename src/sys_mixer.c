/*
 * This file is part of mod-system-control.
 */

#include "sys_mixer.h"
#include "serial.h"

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>

static volatile bool sys_mixer_thread_running;
static sem_t sys_mixer_semaphore;
static pthread_t sys_mixer_thread;
static bool s_debug;

// defined in reply.c
/*static*/ bool execute_ignoring_output(struct sp_port* const serialport, const char* argv[], const bool debug);

typedef struct amixer_msg {
    bool valid;
    bool input;
    char channel;
    char control[9]; /* biggest possible is "exppedal" */
    char value[8];
} amixer_msg;

static struct amixer_msg last_amixer_msg;
static pthread_mutex_t last_amixer_mutex = PTHREAD_MUTEX_INITIALIZER;

static void handle_postponed_message(const amixer_msg* const msg)
{
    // headphone mode
    if (msg->channel == 'h')
    {
        const char* argv[] = { "mod-amixer", "hp", "xvol", msg->value, NULL };

        execute_ignoring_output(NULL, argv, s_debug);
    }
    // gain mode
    else
    {
        const char* const io = msg->input ? "in" : "out";
        const char channelstr[2] = { msg->channel, '\0' };
        const char* argv[] = { "mod-amixer", io, channelstr, "xvol", msg->value, NULL };

        execute_ignoring_output(NULL, argv, s_debug);
    }
}

static void* postponed_messages_thread_run(void* const arg)
{
    struct amixer_msg local_amixer_msg;

    while (sys_mixer_thread_running)
    {
        pthread_mutex_lock(&last_amixer_mutex);

        if (last_amixer_msg.valid)
        {
            local_amixer_msg = last_amixer_msg;
            last_amixer_msg.valid = false;
        }

        pthread_mutex_unlock(&last_amixer_mutex);

        if (local_amixer_msg.valid)
        {
            local_amixer_msg.valid = false;
            handle_postponed_message(&local_amixer_msg);
        }

        sem_wait(&sys_mixer_semaphore);
    }

    return NULL;

    // unused
    (void)arg;
}

void sys_mixer_setup(const bool debug)
{
    s_debug = debug;
    sys_mixer_thread_running = true;
    sem_init(&sys_mixer_semaphore, 0, 0);
    pthread_create(&sys_mixer_thread, NULL, postponed_messages_thread_run, NULL);
}

void sys_mixer_destroy()
{
    sys_mixer_thread_running = false;
    sem_post(&sys_mixer_semaphore);
    pthread_join(sys_mixer_thread, NULL);
    sem_destroy(&sys_mixer_semaphore);
}

void sys_mixer_gain(bool input, char channel, const char* value)
{
    pthread_mutex_lock(&last_amixer_mutex);

    // trigger previously cached value if does not match current one
    if (last_amixer_msg.valid && (last_amixer_msg.input != input ||
                                  last_amixer_msg.channel != channel ||
                                  strcmp(last_amixer_msg.control, "xvol") != 0))
    {
        handle_postponed_message(&last_amixer_msg);
    }

    // cache request for later handling
    last_amixer_msg.valid = true;
    last_amixer_msg.input = input;
    last_amixer_msg.channel = channel;
    strcpy(last_amixer_msg.control, "xvol");
    strncpy(last_amixer_msg.value, value, sizeof(last_amixer_msg.value)-1);

    sem_post(&sys_mixer_semaphore);
    pthread_mutex_unlock(&last_amixer_mutex);

    if (s_debug)
        printf("%s: postponing amixer gain value set\n", __func__);
}

void sys_mixer_headphone(const char* value)
{
    pthread_mutex_lock(&last_amixer_mutex);

    // trigger previously cached value if does not match current one
    if (last_amixer_msg.valid && (last_amixer_msg.channel != 'h' ||
                                  strcmp(last_amixer_msg.control, "xvol") != 0))
    {
        handle_postponed_message(&last_amixer_msg);
    }

    // cache request for later handling
    last_amixer_msg.valid = true;
    last_amixer_msg.input = false;
    last_amixer_msg.channel = 'h';
    strcpy(last_amixer_msg.control, "xvol");
    strncpy(last_amixer_msg.value, value, sizeof(last_amixer_msg.value)-1);

    sem_post(&sys_mixer_semaphore);
    pthread_mutex_unlock(&last_amixer_mutex);

    if (s_debug)
        printf("%s: postponing amixer hp gain value set\n", __func__);
}
