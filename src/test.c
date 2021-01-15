/*
 * This file is part of mod-system-control.
 */

#include "serial_io.h"
#include "serial_rw.h"
#include "reply.h"

#define DEBUG
#define _DEBUG
#include <assert.h>

// #define _GNU_SOURCE
// #include <stdbool.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

int main(int argc, char* argv[])
{
    struct sp_port* const serialport_sys = serial_open("sys", 0);
    struct sp_port* const serialport_hmi = serial_open("hmi", 0);

    if (serialport_sys == NULL || serialport_hmi == NULL)
        return 1;

    char buf[0xff];
    int ret;

    // TEST: initial read must return no data
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);

    ret = serial_read_msg_until_zero(serialport_hmi, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);

#if 0
    uint8_t msg_size;
    size_t written;

    written = snprintf(buf, 0xff, "%s", MOD_SYSTEM_SERIAL_CMD_BT_INFO);
    assert(written == MOD_SYSTEM_SERIAL_CMD_SIZE);

//     written = loribu_write(serialport->bread.loribu, (uint8_t*)buf, written + 1);
//     assert(written == MOD_SYSTEM_SERIAL_CMD_SIZE + 1);

    bzero(buf, sizeof(buf));

    msg_size = serial_read_msg_until_zero(serialport, buf);
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
//     written = loribu_write(serialport->bread.loribu, (uint8_t*)buf, written + 1);

    msg_size = serial_read_msg_until_zero(serialport, buf);
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
#endif

    serial_close(serialport_sys);
    serial_close(serialport_hmi);
    return 0;

    // unused
    (void)argc;
    (void)argv;
}
