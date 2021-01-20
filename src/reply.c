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

static bool execute_and_write_output_resp(struct sp_port* serialport, const char* argv[])
{
//     char cmdbuf[0xff];

//     if (execute_and_get_output(cmdbuf, argv))
//         return write_or_close(serialport, "r 0");

    return write_or_close(serialport, CMD_RESPONSE " -1");
}

bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff])
{
    if (strncmp(msg, CMD_SYS_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* argvs = msg + SYS_CMD_ARG_START;

        // parsing arguments
        const char *io, *value;
        char channel[2];

        // io 0 = in, 1 = out
        io = *argvs++ == '0' ? "in" : "out";
        argvs++;
        // channel 1, 2 or 0 for both
        channel[0] = *argvs++;
        channel[1] = '\0';
        argvs++;
        // mixer value
        value = argvs;

        const char* argv[] = { "mod-amixer", io, channel, value, NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_HP_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-amixer", "hp", "xvol", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_BT_STATUS, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "hmi", NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_BT_DISCOVERY, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "discovery", NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_SYSTEMCTL, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "systemctl", "is-active", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_VERSION, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-version", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    if (strncmp(msg, CMD_SYS_SERIAL, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "cat", "/var/cache/mod/tag", NULL };

        return execute_and_write_output_resp(serialport, argv);
    }

    fprintf(stderr, "%s: unknown message '%s'\n", __func__, msg);
    return true;
}
