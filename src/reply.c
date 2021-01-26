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

// "sys_ver 07 version" -> "version"
#define SYS_CMD_ARG_START (_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2)

static bool execute_ignoring_output(struct sp_port* serialport, const char* argv[], const bool debug)
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

static bool execute_and_write_output_resp(struct sp_port* serialport, const char* argv[], const bool debug)
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

bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff], const bool debug)
{
    if (strncmp(msg, CMD_SYS_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* argvs = msg + SYS_CMD_ARG_START;

        // parsing arguments
        const char *io, *value = NULL;
        char channel[2];

        // io 0 = in, 1 = out
        io = *argvs++ == '0' ? "in" : "out";
        argvs++;

        // channel 1, 2 or 0 for both
        channel[0] = *argvs++;
        channel[1] = '\0';

        // mixer value (optional)
        if (*argvs != '\0')
            value = ++argvs;

        const char* argv[] = { "mod-amixer", io, channel, "xvol", value, NULL };

        return value == NULL ? execute_and_write_output_resp(serialport, argv, debug)
                             : execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_HP_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;
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
