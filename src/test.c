/*
 * This file is part of mod-system-control.
 */

#include "serial_io.h"
#include "serial_rw.h"
#include "reply.h"

#include "../mod-controller-proto/mod-protocol.h"

#ifndef DEBUG
 #define DEBUG
#endif
#define _DEBUG
#include <assert.h>

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void update_syscmd_size(char cmdbuf[0xff]);
static void test_hmi_command(struct sp_port* hmi, struct sp_port* sys, const char* cmd, const char* resp);

int main(int argc, char* argv[])
{
    const char* serialA;
    const char* serialB;
    int baudrate;

    // real version
    if (sp_get_lib_version_string() != NULL)
    {
        if (argc <= 3)
        {
            fprintf(stdout, "Usage: %s <serial-device-a> <serial-device-b> <speed>\n", argv[0]);
            return EXIT_FAILURE;
        }

        // parse arguments
        serialA = argv[1];
        serialB = argv[2];
        baudrate = atoi(argv[3]);
    }
    // fake version
    else
    {
        serialA = "sys";
        serialB = "hmi";
        baudrate = 0;
    }

    struct sp_port* const serialport_sys = serial_open(serialA, baudrate);
    struct sp_port* const serialport_hmi = serial_open(serialB, baudrate);

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
    // test random gibberish and that we can still proceed

    printf("TEST: write writing random gibberish from hmi side\n");
    assert(write_or_close(serialport_hmi, "Lorem ipsum dolor sit amet"));
    assert(write_or_close(serialport_hmi, "consectetur adipiscing elit"));
    assert(write_or_close(serialport_hmi, "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua"));
    assert(write_or_close(serialport_hmi, "Ut enim ad minim veniam"));
    assert(write_or_close(serialport_hmi, "quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat"));
    assert(write_or_close(serialport_hmi, "Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur"));
    assert(write_or_close(serialport_hmi, "Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum"));
    printf("\n");

    for (;;)
    {
        printf("TEST: read gibberish from sys side\n");
        ret = serial_read_msg_until_zero(serialport_sys, buf);
        assert(ret == SP_READ_ERROR_INVALID_DATA || ret == SP_READ_ERROR_NO_DATA);
        printf("\n");

        if (ret == SP_READ_ERROR_NO_DATA)
            break;

        printf("TEST: fixup serial after gibberish\n");
        assert(serial_read_ignore_until_zero(serialport_sys) == 0);
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
    // now test all commands and their expected output

    char cmdbuf[0xff], respbuf[0xff];

    // get (in, channel 1)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 1, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 8);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get (in, channel 2)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 2, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 8);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get (in, both channels)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 8);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get (out, channel 1)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 1, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 0);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get (out, channel 2)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 1, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 0);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get (out, both channels)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 0);
    cmdbuf[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 5] = '\0';
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (in, channel 1)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 1, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (in, channel 2)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 2, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (in, both channels)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 0, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (out, channel 1)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 1, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (out, channel 2)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 2, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set (out, both channels)
    snprintf(cmdbuf, 0xff-1, CMD_SYS_GAIN, 0, 1, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // get hp
    snprintf(cmdbuf, 0xff-1, CMD_SYS_HP_GAIN, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_INT, 0, 12);
    cmdbuf[_CMD_SYS_LENGTH] = '\0';
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // set hp
    snprintf(cmdbuf, 0xff-1, CMD_SYS_HP_GAIN, 0, 0.0);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_AMIXER_SAVE);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_BT_STATUS);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "Unavailable||(none)");
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_BT_DISCOVERY);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_NONE, 0);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_SYSTEMCTL, 0, "jack2");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "inactive");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_SYSTEMCTL, 0, "ssh");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "active");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_VERSION, 0, "version");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "v1.10.0");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_VERSION, 0, "restore");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "625f35a6d03643f138a73fa1e2a8");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_VERSION, 0, "system");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "344178712cd4084bd1260a9e5e5ce0fa23e16a0e");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_VERSION, 0, "controller");
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "ab7a1f5af57b05249530efb7ea645119ca0a7df5");
    update_syscmd_size(cmdbuf);
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    snprintf(cmdbuf, 0xff-1, CMD_SYS_SERIAL);
    snprintf(respbuf, 0xff-1, CMD_RESPONSE_STR, 0, "MDW01D01-00001");
    test_hmi_command(serialport_hmi, serialport_sys, cmdbuf, respbuf);

    // --------------------------------------------------------------------------------------------

    serial_close(serialport_sys);
    serial_close(serialport_hmi);
    return EXIT_SUCCESS;

    // unused
    (void)argc;
    (void)argv;
}

static void update_syscmd_size(char cmdbuf[0xff])
{
    static const char hexadecimals[] = {
        '0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'
    };
    const size_t len = strlen(cmdbuf + (_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2));
    cmdbuf[_CMD_SYS_LENGTH + 1] = hexadecimals[len / 16];
    cmdbuf[_CMD_SYS_LENGTH + 2] = hexadecimals[len & 15];
}

static void test_hmi_command(struct sp_port* const serialport_hmi,
                             struct sp_port* const serialport_sys,
                             const char* const cmd,
                             const char* const resp)
{
    char buf[0xff];
    int ret;

    printf("TEST: write '%s' from hmi side\n", cmd);
    assert(write_or_close(serialport_hmi, cmd));
    printf("\n");

    printf("TEST: read '%s' from sys side\n", cmd);
    ret = serial_read_msg_until_zero(serialport_sys, buf);
    assert(ret == (int)strlen(cmd));
    assert(strcmp(buf, cmd) == 0);
    printf("\n");

    printf("TEST: reply to '%s'\n", cmd);
    assert(parse_and_reply_to_message(serialport_sys, buf));
    printf("\n");

    printf("TEST: read '%s' reply\n", cmd);
    assert(serial_read_response(serialport_hmi, buf));
    printf("TEST: read '%s' reply -> '%s' vs '%s'\n", cmd, buf, resp);
    assert(strcmp(buf, resp) == 0);
    printf("\n");
}
