/*
 * This file is part of mod-system-control.
 */

#include "../loribu/src/loribu.c"

#define DEBUG
#define _DEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define SP_BUFFER_SIZE 4096

// definitions from libserialport API
enum sp_return {
    SP_OK = 0,
    SP_ERR_ARG = -1,
    SP_ERR_FAIL = -2
};

struct sp_port_buffer {
    uint8_t* buffer;
    loribu_t* loribu;
};

struct sp_port {
    struct sp_port_buffer bread;
    struct sp_port_buffer bwrite;
};

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

// fake implementation of libserialport
static enum sp_return sp_blocking_read(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms)
{
    const uint32_t read = loribu_read(port->bread.loribu, buf, count);
    printf("sp_blocking_read(%p, %s, %lu) -> %u\n", port, (const char*)buf, count, read);
    return read == count ? read : -SP_ERR_FAIL;

    // unused
    (void)timeout_ms;
}

static enum sp_return sp_nonblocking_write(struct sp_port *port, const void *buf, size_t count)
{
    const uint32_t written = loribu_write(port->bwrite.loribu, buf, count);
    printf("sp_nonblocking_write(%p, %s, %lu) -> %u\n", port, (const char*)buf, count, written);
    return written == count ? written : -SP_ERR_FAIL;
}

static struct sp_port* serial_open(void)
{
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
    return serialport;

fail_serialport_malloc:
    sp_port_buffer_cleanup(&bwrite);

fail_bwrite_init:
    sp_port_buffer_cleanup(&bread);

fail_bread_init:
    return NULL;
}

static void serial_close(struct sp_port* const serialport)
{
    sp_port_buffer_cleanup(&serialport->bwrite);
    sp_port_buffer_cleanup(&serialport->bread);
    free(serialport);
}

#define MOD_SYSTEM_CONTROL_TESTS
#include "../system-control.c"

int main(int argc, char* argv[])
{
    struct sp_port* const serialport = serial_open();
    char buf[0xff];
    uint8_t msg_size;
    size_t written;

    if (serialport == NULL)
        return 1;

    written = snprintf(buf, 0xff, "%s", MOD_SYSTEM_SERIAL_CMD_BT_INFO);
    assert(written == MOD_SYSTEM_SERIAL_CMD_SIZE);

    written = loribu_write(serialport->bread.loribu, (uint8_t*)buf, written + 1);
    assert(written == MOD_SYSTEM_SERIAL_CMD_SIZE + 1);

    bzero(buf, sizeof(buf));

    msg_size = serial_read_until_zero(serialport, buf);
    printf("message read %u bytes\n", msg_size);

    if (msg_size != 0)
    {
        printf("message read ok\n");

        if (parse_and_reply_to_message(serialport, buf))
        {
            printf("message reply ok\n");
        }
        else
        {
            printf("message reply fail\n");
        }
    }
    else
    {
        printf("message read fail\n");
    }

    bzero(buf, sizeof(buf));

    written = snprintf(buf, 0xff, "%s %02x testing", MOD_SYSTEM_SERIAL_CMD_BT_INFO, (unsigned int)strlen("testing"));
    written = loribu_write(serialport->bread.loribu, (uint8_t*)buf, written + 1);

    msg_size = serial_read_until_zero(serialport, buf);
    printf("message read %u bytes\n", msg_size);

    if (msg_size != 0)
    {
        printf("message read ok\n");

        if (parse_and_reply_to_message(serialport, buf))
        {
            printf("message reply ok\n");
        }
        else
        {
            printf("message reply fail\n");
        }
    }
    else
    {
        printf("message read fail\n");
    }

    serial_close(serialport);
    return 0;

    // unused
    (void)argc;
    (void)argv;
}
