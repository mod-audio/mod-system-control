/*
 * This file is part of mod-system-control.
 */

#include "serial_io.h"
#include "serial_rw.h"
#include "reply.h"

#ifndef DEBUG
 #define DEBUG
#endif
#define _DEBUG
#include <assert.h>

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>

static void test_hmi_command();

int main(int argc, char* argv[])
{
    struct sp_port* const serialport_sys = serial_open("sys", 0);
    struct sp_port* const serialport_hmi = serial_open("hmi", 0);

    if (serialport_sys == NULL || serialport_hmi == NULL)
        return 1;

    char buf[0xff];
    int ret;

    // --------------------------------------------------------------------------------------------
    // initial read must return no data

    printf("TEST: initial read must return no data (sys)\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    printf("TEST: initial read must return no data (hmi)\n");
    ret = serial_read_msg_until_zero(serialport_hmi, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read simple valid message

    printf("TEST: write sys_ver from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 07 version"));
    printf("\n");

    printf("TEST: read sys_ver from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == strlen("sys_ver 07 version"));
    assert(strcmp(buf, "sys_ver 07 version") == 0);
    printf("\n");

    printf("TEST: reply to sys_ver\n");
    assert(parse_and_reply_to_message(serialport_sys, buf));
    printf("\n");

    printf("TEST: read sys_ver reply\n");
    assert(serial_read_response(serialport_hmi, buf));
    assert(strcmp(buf, "r 0 v1.10.0") == 0);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // there should be nothing more to read

    printf("TEST: there should be nothing more to read (sys)\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    printf("TEST: there should be nothing more to read (hmi)\n");
    ret = serial_read_msg_until_zero(serialport_hmi, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read unsupported command

    printf("TEST: write unsupported command from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_fuk"));
    printf("\n");

    printf("TEST: read unsupported command from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == strlen("sys_fuk"));
    assert(strcmp(buf, "sys_fuk") == 0);
    printf("\n");

    printf("TEST: reply to unsupported command\n");
    assert(parse_and_reply_to_message(serialport_sys, buf));
    printf("\n");

    printf("TEST: read unsupported command reply\n");
    assert(serial_read_response(serialport_hmi, buf));
    assert(strcmp(buf, "r -1") == 0);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read valid command but with wrong data length (part 1)

    printf("TEST: write valid command but with wrong data length (part 1) from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 0"));
    printf("\n");

    printf("TEST: read valid command but with wrong data length (part 1) from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read valid command but with wrong data length (part 2)

    printf("TEST: write valid command but with wrong data length (part 2) from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 00"));
    printf("\n");

    printf("TEST: read valid command but with wrong data length (part 2) from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read valid command but with too small data size

    printf("TEST: write valid command but with too small data size from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 04 version"));
    printf("\n");

    printf("TEST: read valid command but with too small data size from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_INVALID_DATA);
    printf("\n");

    printf("TEST: fixup serial after valid command but with too small data size\n");
    assert(serial_read_ignore_until_zero(serialport_sys) == 0);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read valid command but with too big data size

    printf("TEST: write valid command but with too big data size from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 0f version"));
    printf("\n");

    printf("TEST: read valid command but with too big data size from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read valid command but with corrupt data

    printf("TEST: write valid command but with corrupt data from hmi side\n");
    assert(write_or_close(serialport_hmi, "sys_ver 04 :`[!"));
    printf("\n");

    printf("TEST: read valid command but with corrupt data from sys side\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == strlen("sys_ver 04 :`[!"));
    assert(strcmp(buf, "sys_ver 04 :`[!") == 0);
    printf("\n");

    printf("TEST: reply to valid command but with corrupt data\n");
    assert(parse_and_reply_to_message(serialport_sys, buf));
    printf("\n");

    printf("TEST: read valid command but with corrupt data reply\n");
    assert(serial_read_response(serialport_hmi, buf));
    assert(strcmp(buf, "r -1") == 0);
    printf("\n");

    // --------------------------------------------------------------------------------------------
    // write and read gibberish command

    printf("TEST: write gibberish command\n");
    assert(write_or_close(serialport_hmi, "sys_gibberish"));
    printf("\n");

    printf("TEST: read gibberish command\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_INVALID_DATA);
    printf("\n");

    printf("TEST: fixup serial after valid command but with wrong data size\n");
    assert(serial_read_ignore_until_zero(serialport_sys) == 0);
    printf("\n");

    // --------------------------------------------------------------------------------------------

    printf("TEST: write writing random gibberish from hmi side\n");
    assert(write_or_close(serialport_hmi, "Lorem ipsum dolor sit amet"));
    assert(write_or_close(serialport_hmi, "consectetur adipiscing elit"));
    assert(write_or_close(serialport_hmi, "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua"));
    assert(write_or_close(serialport_hmi, "Ut enim ad minim veniam"));
    assert(write_or_close(serialport_hmi, "quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat"));
    assert(write_or_close(serialport_hmi, "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur"));
    assert(write_or_close(serialport_hmi, "Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum"));
    printf("\n");

    for (int i=0;; ++i)
    {
        printf("TEST: read gibberish from sys side\n");
        ret = serial_read_msg_until_zero(serialport_sys, buf);
        // for sure there are more than 48 attempts to decode stuff
        if (i < 48) {
            assert(ret == SP_READ_ERROR_INVALID_DATA);
        // but we expect data to end at some point
        } else {
            assert(ret == SP_READ_ERROR_INVALID_DATA || ret == SP_READ_ERROR_NO_DATA);
            if (ret == SP_READ_ERROR_NO_DATA)
                break;
        }
        printf("\n");

        printf("TEST: reply to gibberish\n");
        assert(parse_and_reply_to_message(serialport_sys, buf));
        printf("\n");

        printf("TEST: read gibberish command reply\n");
        assert(serial_read_response(serialport_hmi, buf));
        assert(strcmp(buf, "r -1") == 0);
        printf("\n");
    }

    // --------------------------------------------------------------------------------------------
    // there should be nothing more to read

    printf("TEST: there should be nothing more to read (sys)\n");
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    printf("TEST: there should be nothing more to read (hmi)\n");
    ret = serial_read_msg_until_zero(serialport_hmi, buf);
    assert(ret == SP_READ_ERROR_NO_DATA);
    printf("\n");

    // --------------------------------------------------------------------------------------------

    test_hmi_command();

    // --------------------------------------------------------------------------------------------

    sp_close(serialport_sys);
    sp_close(serialport_hmi);
    return 0;

    // unused
    (void)argc;
    (void)argv;
}

static void test_hmi_command()
{
}
