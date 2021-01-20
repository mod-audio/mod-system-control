/*
 * This file is part of mod-system-control.
 */

#include "serial_io.h"

#define _GNU_SOURCE
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

struct sp_port* serial_open(const char* const serial, const int baudrate)
{
    enum sp_return ret;
    struct sp_port* serialport;
    const char* resolvedserial = serial;

    // check that serial exists
    struct stat stat;
    if (lstat(serial, &stat) != 0)
    {
        fprintf(stderr, "%s failed, serial device '%s' does not exist\n", __func__, serial);
        return NULL;
    }

    // check for symbolic link
    if (S_ISLNK(stat.st_mode))
    {
        const size_t bufsiz = stat.st_size > 0 ? (size_t)stat.st_size + 1 : PATH_MAX;
        void* const buf = malloc(bufsiz);

        if (buf == NULL)
        {
            fprintf(stderr, "%s failed, out of memory\n", __func__);
            return NULL;
        }

        resolvedserial = (const char*)buf;

        if (readlink(serial, buf, bufsiz) == -1)
        {
            fprintf(stderr, "%s failed, could not resolve serial device symlink\n", __func__);
            goto error_free;
        }
    }

    // get serial port
    ret = sp_get_port_by_name(resolvedserial, &serialport);
    if (ret != SP_OK)
    {
        fprintf(stderr, "%s failed, cannot get serial port for device '%s'\n", __func__, resolvedserial);
        goto error_free;
    }

    // open serial port
    ret = sp_open(serialport, SP_MODE_READ_WRITE);
    if (ret != SP_OK)
    {
        fprintf(stderr, "%s failed, cannot open serial port for device '%s'\n", __func__, resolvedserial);
        goto error_with_serialport;
    }

    // disable XON/XOFF flow control
    sp_set_xon_xoff(serialport, SP_XONXOFF_DISABLED);

    // configure serial port
    sp_set_baudrate(serialport, baudrate);

    // cleanup
    if (resolvedserial != serial)
        free((char*)resolvedserial);

    return serialport;

error_with_serialport:
    sp_free_port(serialport);

error_free:
    if (resolvedserial != serial)
        free((char*)resolvedserial);

    return NULL;
}

void serial_close(struct sp_port* const serialport)
{
    sp_close(serialport);
    sp_free_port(serialport);
}
