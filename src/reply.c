/*
 * This file is part of mod-system-control.
 */

#include "reply.h"
#include "serial_rw.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TODO put this in spec file
#define MOD_SYSTEM_SERIAL_CMD_PREFIX "sys_"
#define MOD_SYSTEM_SERIAL_CMD_SIZE  7 /* "cmd_xyz" */
#define MOD_SYSTEM_SERIAL_DATA_SIZE 2 /* "ff" max */

#define MOD_SYSTEM_SERIAL_CMD_VERSION MOD_SYSTEM_SERIAL_CMD_PREFIX "ver"
#define MOD_SYSTEM_SERIAL_CMD_BT_INFO MOD_SYSTEM_SERIAL_CMD_PREFIX "bti"
#define MOD_SYSTEM_SERIAL_CMD_BT_DISC MOD_SYSTEM_SERIAL_CMD_PREFIX "btd"

bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff])
{
    // WIP
    if (strncmp(msg, MOD_SYSTEM_SERIAL_CMD_VERSION, MOD_SYSTEM_SERIAL_CMD_SIZE) == 0)
    {
        fprintf(stdout, "%s: replying version\n", __func__);

        const char* const resp = "v1.10.1\n";
        return write_or_close(serialport, resp);
    }

    if (strncmp(msg, MOD_SYSTEM_SERIAL_CMD_BT_INFO, MOD_SYSTEM_SERIAL_CMD_SIZE) == 0)
    {
        fprintf(stdout, "%s: replying bluetooth info\n", __func__);

        const char* const resp = "this,that,what\n";
        return write_or_close(serialport, resp);
    }

    if (strncmp(msg, MOD_SYSTEM_SERIAL_CMD_BT_DISC, MOD_SYSTEM_SERIAL_CMD_SIZE) == 0)
    {
        fprintf(stdout, "%s: replying bluetooth discovery\n", __func__);

        const char* const resp = "ok\n";
        return write_or_close(serialport, resp);
    }

    fprintf(stderr, "%s: unknown message '%s'\n", __func__, msg);
    return true;
}
