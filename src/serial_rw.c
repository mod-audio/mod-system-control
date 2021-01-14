/*
 * This file is part of mod-system-control.
 */

#include "serial_rw.h"

#define _GNU_SOURCE
#include <errno.h>
#include <limits.h>
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

sp_read_error_status serial_read_msg_until_zero(struct sp_port* const serialport, char buf[0xff])
{
    unsigned int reading_offset;
    unsigned int total_msg_size;
    enum sp_return ret;

    // read command
    reading_offset = 0;
    ret = sp_blocking_read(serialport, buf, MOD_SYSTEM_SERIAL_CMD_SIZE + 1, 500);

    if (ret < MOD_SYSTEM_SERIAL_CMD_SIZE + 1)
    {
        if (ret != SP_OK && (ret != 1 || buf[0] != '\0'))
            fprintf(stderr, "%s failed, reading command timed out or error, ret %d\n", __func__, ret);
        return 0;
    }

    // check if message is valid
    if (strncmp(buf, MOD_SYSTEM_SERIAL_CMD_PREFIX, strlen(MOD_SYSTEM_SERIAL_CMD_PREFIX)) != 0)
    {
#if 1
        // TESTING we should not print invalid chars
        buf[MOD_SYSTEM_SERIAL_CMD_SIZE] = '\0';
        fprintf(stderr, "%s failed, invalid command '%s' receive\n", __func__, buf);
#else
        fprintf(stderr, "%s failed, invalid command received\n", __func__);
#endif
        return 0;
    }

    // message was read in full (only has command), we can stop here
    if (buf[MOD_SYSTEM_SERIAL_CMD_SIZE] == '\0')
        return MOD_SYSTEM_SERIAL_CMD_SIZE;

    if (buf[MOD_SYSTEM_SERIAL_CMD_SIZE] != ' ')
    {
        fprintf(stderr, "%s failed, command is missing space delimiter\n", __func__);
        return 0;
    }

    // message has more data on it, let's fetch the data size
    reading_offset += MOD_SYSTEM_SERIAL_CMD_SIZE + 1;
    ret = sp_blocking_read(serialport, buf + reading_offset, MOD_SYSTEM_SERIAL_DATA_SIZE + 1, 500);

    if (ret < MOD_SYSTEM_SERIAL_DATA_SIZE + 1)
    {
        fprintf(stderr, "%s failed, reading command data size timed out or error\n", __func__);
        return 0;
    }

    // check that data size is correct
    {
        long int data_size;
        char data_size_str[MOD_SYSTEM_SERIAL_DATA_SIZE + 1];
        for (uint i = 0; i <= MOD_SYSTEM_SERIAL_DATA_SIZE; ++i)
            data_size_str[i] = buf[MOD_SYSTEM_SERIAL_CMD_SIZE + 1 + i];
        data_size_str[MOD_SYSTEM_SERIAL_DATA_SIZE] = '\0';

        data_size = strtol(data_size_str, NULL, 16);
        // LONG_MAX;
        // LONG_MIN;

        if (data_size <= 0 || data_size > 0xff - reading_offset - 1)
        {
            fprintf(stderr, "%s failed, incorrect command data size '%s'\n", __func__, data_size_str);
            return 0;
        }

        // NOTE does not include cmd and size prefix
        total_msg_size = (uint8_t)data_size;
    }

    // read the full message now
    reading_offset += MOD_SYSTEM_SERIAL_DATA_SIZE + 1;
    ret = sp_blocking_read(serialport, buf + reading_offset, total_msg_size + 1U, 500);

    if (ret < (int)total_msg_size)
    {
        fprintf(stderr, "%s failed, reading full message data timed out or error\n", __func__);
        return 0;
    }

    // add cmd and data size for the correct total size
    total_msg_size += MOD_SYSTEM_SERIAL_CMD_SIZE + MOD_SYSTEM_SERIAL_DATA_SIZE + 2U;

    if (buf[total_msg_size] != '\0')
    {
        fprintf(stderr, "%s failed, full message is not null terminated\n", __func__);
        return 0;
    }

    return (uint8_t)total_msg_size;
}

bool serial_read_ignore_until_zero(struct sp_port* serialport)
{
    // TODO
    return true;
}

bool write_or_close(struct sp_port* serialport, const char* const resp)
{
    errno = 0;
    if (sp_nonblocking_write(serialport, resp, strlen(resp)+1) == -SP_ERR_FAIL && errno == EIO)
        return false;
    return true;
}
