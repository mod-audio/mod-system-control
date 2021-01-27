/*
 * This file is part of mod-system-control.
 */

#include "reply.h"
#include "cli.h"
#include "serial_rw.h"

#include "../mod-controller-proto/mod-protocol.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

// "sys_ver 07 version" -> "version"
#define SYS_CMD_ARG_START (_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2)

static bool execute_ignoring_output(struct sp_port* const serialport, const char* argv[], const bool debug)
{
    if (execute(argv, debug))
    {
        if (debug)
            printf("execute(%p) completed successly\n", argv);

        return write_or_close(serialport, "r 0");
    }

    if (debug)
        printf("execute(%p) failed\n", argv);

    return write_or_close(serialport, "r -1");
}

static bool execute_and_write_output_resp(struct sp_port* const serialport, const char* argv[], const bool debug)
{
    char cmdbuf[0xff + 4];

    if (execute_and_get_output(cmdbuf, argv, debug))
    {
        memmove(cmdbuf+4, cmdbuf, strlen(cmdbuf)+1);
        cmdbuf[0] = 'r';
        cmdbuf[1] = ' ';
        cmdbuf[2] = '0';
        cmdbuf[3] = ' ';

        if (debug)
            printf("execute_and_write_output_resp(%p) completed successly, responding with '%s'\n", argv, cmdbuf);

        return write_or_close(serialport, cmdbuf);
    }

    if (debug)
        printf("execute_and_write_output_resp(%p) failed\n", argv);

    return write_or_close(serialport, "r -1");
}

/* TODO this could likely go into its own separate file
 * something for later, when we add Duo/DuoX support and the code becomes more complex
 */
typedef struct amixer_msg {
    bool valid;
    bool input;
    char channel;
    char control[9]; /* biggest possible is "exppedal" */
    char value[8];
} amixer_msg;
static struct amixer_msg last_amixer_msg;
static pthread_mutex_t last_amixer_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t last_amixer_semaphore;
static pthread_t last_amixer_thread;
static volatile bool last_amixer_thread_running;
static bool s_debug;

static void handle_postponed_message(struct sp_port* const serialport, const amixer_msg* const msg, const bool debug)
{
    // headphone mode
    if (msg->channel == 'h')
    {
        const char* argv[] = { "mod-amixer", "hp", "xvol", msg->value, NULL };

        execute_ignoring_output(serialport, argv, debug);
    }
    // gain mode
    else
    {
        const char* const io = msg->input ? "in" : "out";
        const char channelstr[2] = { msg->channel, '\0' };
        const char* argv[] = { "mod-amixer", io, channelstr, "xvol", msg->value, NULL };

        execute_ignoring_output(serialport, argv, debug);
    }
}

static void* postponed_messages_thread_run(void* const arg)
{
    struct sp_port* const serialport = (struct sp_port*)arg;
    struct amixer_msg local_amixer_msg;

    while (last_amixer_thread_running)
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
            handle_postponed_message(serialport, &local_amixer_msg, s_debug);
        }

        sem_wait(&last_amixer_semaphore);
    }

    return NULL;
}

void create_postponed_messages_thread(struct sp_port* serialport, const bool debug)
{
    s_debug = debug;
    last_amixer_thread_running = true;
    sem_init(&last_amixer_semaphore, 0, 0);
    pthread_create(&last_amixer_thread, NULL, postponed_messages_thread_run, serialport);
}

void destroy_postponded_messages_thread(void)
{
    last_amixer_thread_running = false;
    sem_post(&last_amixer_semaphore);
    pthread_join(last_amixer_thread, NULL);
    sem_destroy(&last_amixer_semaphore);
}

bool parse_and_reply_to_message(struct sp_port* const serialport, char msg[0xff], const bool debug)
{
    if (strncmp(msg, CMD_SYS_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* argvs = msg + SYS_CMD_ARG_START;

        // parsing arguments

        // io 0 = in, 1 = out
        const bool input = *argvs++ == '0';
        const char* const io = input ? "in" : "out";
        argvs++;

        // channel 1, 2 or 0 for both
        const char channel = *argvs++;
        const char channelstr[2] = { channel, '\0' };

        // mixer value (optional)
        const char* value = NULL;

        if (*argvs != '\0')
        {
            value = ++argvs;

            pthread_mutex_lock(&last_amixer_mutex);

            // trigger previously cached value if does not match current one
            if (last_amixer_msg.valid && (last_amixer_msg.input != input ||
                                          last_amixer_msg.channel != channel ||
                                          strcmp(last_amixer_msg.control, "xvol") != 0))
            {
                handle_postponed_message(serialport, &last_amixer_msg, debug);
            }

            // cache request for later handling
            last_amixer_msg.valid = true;
            last_amixer_msg.input = input;
            last_amixer_msg.channel = channel;
            strcpy(last_amixer_msg.control, "xvol");
            strncpy(last_amixer_msg.value, value, sizeof(last_amixer_msg.value)-1);

            sem_post(&last_amixer_semaphore);
            pthread_mutex_unlock(&last_amixer_mutex);

            if (debug)
                printf("%s: postponing amixer gain value set\n", __func__);

            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", io, channelstr, "xvol", value, NULL };

        return value == NULL ? execute_and_write_output_resp(serialport, argv, debug)
                             : execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_HP_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            pthread_mutex_lock(&last_amixer_mutex);

            // trigger previously cached value if does not match current one
            if (last_amixer_msg.valid && (last_amixer_msg.channel != 'h' ||
                                          strcmp(last_amixer_msg.control, "xvol") != 0))
            {
                handle_postponed_message(serialport, &last_amixer_msg, debug);
            }

            // cache request for later handling
            last_amixer_msg.valid = true;
            last_amixer_msg.input = false;
            last_amixer_msg.channel = 'h';
            strcpy(last_amixer_msg.control, "xvol");
            strncpy(last_amixer_msg.value, value, sizeof(last_amixer_msg.value)-1);

            sem_post(&last_amixer_semaphore);
            pthread_mutex_unlock(&last_amixer_mutex);

            if (debug)
                printf("%s: postponing amixer hp gain value set\n", __func__);

            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", "hp", "xvol", value, NULL };

        return value == NULL ? execute_and_write_output_resp(serialport, argv, debug)
                             : execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_AMIXER_SAVE, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-amixer", "save", NULL };

        return execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_BT_STATUS, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "hmi", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_BT_DISCOVERY, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "discovery", NULL };

        return execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_SYSTEMCTL, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "systemctl", "is-active", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_VERSION, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-version", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_SERIAL, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "cat", "/var/cache/mod/tag", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    fprintf(stderr, "%s: unknown message '%s'\n", __func__, msg);
    return write_or_close(serialport, "r -1");
}
