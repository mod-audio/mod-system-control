/*
 * This file is part of mod-system-control.
 */

#include "serial.h"
#include "../loribu/src/loribu.h"

#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__clang__)
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#elif defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#define SP_BUFFER_SIZE 4096

/* ----------------------------------------------------------------------------------------------------------------- */

struct sp_port_buffer {
    uint8_t* buffer;
    loribu_t* loribu;
};

struct sp_port {
    struct sp_port_buffer bread;
    struct sp_port_buffer bwrite;
    struct sp_port* otherside;
};

/* ----------------------------------------------------------------------------------------------------------------- */

static bool sp_port_buffer_init(struct sp_port_buffer* const b)
{
    uint8_t* const buffer = malloc(SP_BUFFER_SIZE);

    if (buffer == NULL)
        goto fail_malloc;

    loribu_t* const loribu = loribu_create(buffer, SP_BUFFER_SIZE);

    if (loribu == NULL)
        goto fail_loribu_create;

    b->buffer = buffer;
    b->loribu = loribu;
    return true;

fail_loribu_create:
    free(buffer);

fail_malloc:
    return false;
}

static void sp_port_buffer_cleanup(struct sp_port_buffer* const b)
{
    loribu_destroy(b->loribu);
    free(b->buffer);
}

/* ----------------------------------------------------------------------------------------------------------------- */

static struct sp_port* serial_sys = NULL;
static struct sp_port* serial_hmi = NULL;

struct sp_port* serial_open(const char* serial, int baudrate)
{
    if (strcmp(serial, "sys") == 0)
    {
        if (serial_sys != NULL)
        {
            fprintf(stderr, "%s: SYS serial already open", __func__);
            return NULL;
        }
    }
    else if (strcmp(serial, "hmi") == 0)
    {
        if (serial_hmi != NULL)
        {
            fprintf(stderr, "%s: HMI serial already open", __func__);
            return NULL;
        }
    }
    else
    {
        fprintf(stderr, "%s: invalid serial to open", __func__);
        return NULL;
    }

    struct sp_port_buffer bread;
    struct sp_port_buffer bwrite;

    if (! sp_port_buffer_init(&bread))
        goto fail_bread_init;

    if (! sp_port_buffer_init(&bwrite))
        goto fail_bwrite_init;

    struct sp_port* const serialport = (struct sp_port*)malloc(sizeof(struct sp_port));

    if (serialport == NULL)
        goto fail_serialport_malloc;

    serialport->bread = bread;
    serialport->bwrite = bwrite;

    if (strcmp(serial, "sys") == 0)
    {
        serialport->otherside = serial_hmi;
        serial_sys = serialport;

        if (serial_hmi != NULL)
            serial_hmi->otherside = serialport;
    }
    else
    {
        serialport->otherside = serial_sys;
        serial_hmi = serialport;

        if (serial_sys != NULL)
            serial_sys->otherside = serialport;
    }

    return serialport;

fail_serialport_malloc:
    sp_port_buffer_cleanup(&bwrite);

fail_bwrite_init:
    sp_port_buffer_cleanup(&bread);

fail_bread_init:
    return NULL;
}

void serial_close(struct sp_port* serialport)
{
    sp_port_buffer_cleanup(&serialport->bwrite);
    sp_port_buffer_cleanup(&serialport->bread);
    free(serialport);
}

/* ----------------------------------------------------------------------------------------------------------------- */

enum sp_return sp_blocking_read(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms)
{
    const uint32_t read = loribu_read(port->bread.loribu, buf, count);
//     printf("sp_blocking_read(%p, %s, %lu) -> %u\n", port, (const char*)buf, count, read);
    return read == count ? (int)read
                         : (read == 0 ? SP_OK : SP_ERR_FAIL);
}

enum sp_return sp_nonblocking_write(struct sp_port *port, const void *buf, size_t count)
{
    const uint32_t written = loribu_write(port->bwrite.loribu, buf, count);
    printf("sp_nonblocking_write(%p, %s, %lu) -> %u\n", port, (const char*)buf, count, written);
    return written == count ? (int)written : SP_ERR_FAIL;
}

/* ----------------------------------------------------------------------------------------------------------------- */

#if defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
# pragma GCC diagnostic pop
#endif

#include "../loribu/src/loribu.c"
