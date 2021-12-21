/*
 * This file is part of mod-system-control.
 */

#include "serial_rw.h"

#include "../mod-controller-proto/mod-protocol.h"

#define _GNU_SOURCE
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// in ms
#define SP_BLOCKING_READ_TIMEOUT 20

static inline int imax(const int a, const int b)
{
    return a > b ? a : b;
}

sp_read_error_status serial_read_msg_until_zero(struct sp_port* const serialport, char buf[0xff], const bool debug)
{
    unsigned int reading_offset;
    unsigned int total_msg_size;
    enum sp_return ret;

    // read command
    reading_offset = 0;
    ret = sp_blocking_read(serialport, buf, _CMD_SYS_LENGTH + 1, SP_BLOCKING_READ_TIMEOUT);

    /*
    fprintf(stderr, "%s sp_blocking_read first read ret %d |%c|%c|%c|%c|\n", __func__, ret, buf[0], buf[1], buf[2], buf[3]);
    */

    // shift by 1 byte if message starts with a null byte
    if (ret > 1 && buf[0] == '\0' && buf[1] != '\0')
        memmove(buf, buf+1, --ret);

    if (ret < _CMD_SYS_LENGTH + 1)
    {
        // there was nothing to read
        if (ret == 0)
            return SP_READ_ERROR_NO_DATA;

        // if there is data, see if it is valid
        if (ret > 0)
        {
            // check for all zeros, treat as if we read nothing
            static const char allzeros[_CMD_SYS_LENGTH + 1];

            if (memcmp(buf, allzeros, ret) == 0)
                return SP_READ_ERROR_NO_DATA;

            // if we read the beginning of a valid message, maybe we got cut off, let's check for that
            if (strncmp(buf, _CMD_SYS_PREFIX, strlen(_CMD_SYS_PREFIX)) == 0)
            {
                const enum sp_return oldret = ret;
                ret = sp_blocking_read(serialport,
                                       buf + oldret, _CMD_SYS_LENGTH + 1 - oldret,
                                       imax(SP_BLOCKING_READ_TIMEOUT/10, 1));

                if (ret > 0 && ret + oldret == _CMD_SYS_LENGTH + 1)
                    goto check_valid_cmd;
            }
        }

        if (debug && ret > 0)
        {
            char buf2[0xff];
            memcpy(buf2, buf, ret);
            buf[ret] = 0;
            fprintf(stderr, "%s failed, reading command timed out or error, ret %d, value '%s'\n", __func__, ret, buf2);
        }
        else
        {
            fprintf(stderr, "%s failed, reading command timed out or error, ret %d\n", __func__, ret);
        }
        return SP_READ_ERROR_INVALID_DATA;
    }

check_valid_cmd:
    // check if message is valid
    if (strncmp(buf, _CMD_SYS_PREFIX, strlen(_CMD_SYS_PREFIX)) != 0)
    {
        if (debug)
        {
            buf[_CMD_SYS_LENGTH] = '\0';
            fprintf(stderr, "%s failed, invalid command '%s' received\n", __func__, buf);
        }
        else
        {
            fprintf(stderr, "%s failed, invalid command received\n", __func__);
        }
        return SP_READ_ERROR_INVALID_DATA;
    }

    // message was read in full (only has command), we can stop here
    if (buf[_CMD_SYS_LENGTH] == '\0')
        return _CMD_SYS_LENGTH;

    if (buf[_CMD_SYS_LENGTH] != ' ')
    {
        fprintf(stderr, "%s failed, command is missing space delimiter\n", __func__);
        return SP_READ_ERROR_INVALID_DATA;
    }

    // message has more data on it, let's fetch the data size
    reading_offset += _CMD_SYS_LENGTH + 1;
    ret = sp_blocking_read(serialport, buf + reading_offset, _CMD_SYS_DATA_LENGTH + 1, SP_BLOCKING_READ_TIMEOUT);

    if (ret < _CMD_SYS_DATA_LENGTH + 1)
    {
        if (debug)
        {
            buf[_CMD_SYS_LENGTH + 1 + (ret > 0 ? ret : 0)] = '\0';
            fprintf(stderr, "%s failed, reading command data size timed out or error %i, read msg so far was '%s'\n",
                    __func__, ret, buf);
        }
        else
        {
            fprintf(stderr, "%s failed, reading command data size timed out or error %i\n", __func__, ret);
        }
        return (ret > 0 && buf[reading_offset + ret - 1] == '\0') ? SP_READ_ERROR_NO_DATA
                                                                  : SP_READ_ERROR_INVALID_DATA;
    }

    // check that data size is correct
    {
        long int data_size;
        char data_size_str[_CMD_SYS_DATA_LENGTH + 1];
        for (uint i = 0; i <= _CMD_SYS_DATA_LENGTH; ++i)
            data_size_str[i] = buf[_CMD_SYS_LENGTH + 1 + i];
        data_size_str[_CMD_SYS_DATA_LENGTH] = '\0';

        data_size = strtol(data_size_str, NULL, 16);

        if (data_size <= 0 || data_size > 0xff - reading_offset - 1)
        {
            fprintf(stderr, "%s failed, incorrect command data size '%s'\n", __func__, data_size_str);
            return buf[reading_offset + ret - 1] == '\0' ? SP_READ_ERROR_NO_DATA
                                                         : SP_READ_ERROR_INVALID_DATA;
        }

        // NOTE does not include cmd and size prefix
        total_msg_size = (unsigned int)data_size;
    }

    // read the full message now
    reading_offset += _CMD_SYS_DATA_LENGTH + 1;
    ret = sp_blocking_read(serialport, buf + reading_offset, total_msg_size + 1U, SP_BLOCKING_READ_TIMEOUT);

    if (ret < (int)total_msg_size + 1)
    {
        // if we read a few bytes maybe we got cancelled, try again one more time
        enum sp_return ret2 = 0;
        if (ret > 0)
        {
            ret2 = sp_blocking_read(serialport,
                                    buf + reading_offset + ret, total_msg_size + 1U - ret,
                                    imax(SP_BLOCKING_READ_TIMEOUT/10, 1));

            if (ret2 < 0)
                ret2 = 0;
        }

        if (ret + ret2 < (int)total_msg_size)
        {
            fprintf(stderr, "%s failed, reading full message data timed out or error\n", __func__);
            return buf[reading_offset + ret - 1] == '\0' ? SP_READ_ERROR_NO_DATA
                                                         : SP_READ_ERROR_INVALID_DATA;
        }
    }

    // add cmd and data size for the correct total size
    total_msg_size += _CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2U;

    if (buf[total_msg_size] != '\0')
    {
        fprintf(stderr, "%s failed, full message is not null terminated\n", __func__);
        return SP_READ_ERROR_INVALID_DATA;
    }

    return total_msg_size;
}

sp_read_error_status serial_read_ignore_until_zero(struct sp_port* serialport)
{
    char c;
    enum sp_return ret;

    const int timeout = imax(SP_BLOCKING_READ_TIMEOUT/2,1);

    for (;;)
    {
        ret = sp_blocking_read(serialport, &c, 1, timeout);

        if (ret == 0)
            return SP_READ_ERROR_NO_DATA;
        if (ret != 1)
            return SP_READ_ERROR_INVALID_DATA;
        if (c == '\0')
            return 0;
    }
}

bool write_or_close(struct sp_port* serialport, const char* const msg)
{
    errno = 0;
    if (sp_nonblocking_write(serialport, msg, strlen(msg)+1) == -SP_ERR_FAIL && errno == EIO)
    {
        sp_close(serialport);
        return false;
    }

    return true;
}

// NOTE: DO NOT USE, needed only for tests
bool serial_read_response(struct sp_port* serialport, char buf[0xff])
{
    unsigned int reading_offset;
    enum sp_return ret;

    // read first byte
    reading_offset = 0;
    ret = sp_blocking_read(serialport, buf, 1, SP_BLOCKING_READ_TIMEOUT);

    if (ret != 1 || buf[0] != 'r')
        return false;

    // read resp code
    reading_offset += 1;
    ret = sp_blocking_read(serialport, buf + reading_offset, 2, SP_BLOCKING_READ_TIMEOUT);

    if (ret != 2 || buf[1] != ' ')
        return false;

    // if negative resp code, read one more byte and stop here
    if (buf[2] == '-')
    {
        reading_offset += 2;
        ret = sp_blocking_read(serialport, buf + reading_offset, 2, SP_BLOCKING_READ_TIMEOUT);

        if (ret != 2 || buf[4] != '\0')
            return false;

        /*
        fprintf(stderr, "%s ok with err, full message is \"%s\"\n", __func__, buf);
        */
        return true;
    }

    // read everything byte by byte until zero
    reading_offset += 2;
    for (;;)
    {
        ret = sp_blocking_read(serialport, buf + reading_offset, 1, SP_BLOCKING_READ_TIMEOUT);

        if (ret != 1)
            return false;

        if (buf[reading_offset] == '\0')
        {
            /*
            fprintf(stderr, "%s ok, full message is \"%s\"\n", __func__, buf);
            */
            return true;
        }

        reading_offset += 1;
    }
}
