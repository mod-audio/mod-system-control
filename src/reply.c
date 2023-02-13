/*
 * This file is part of mod-system-control.
 */

#include "reply.h"
#include "cli.h"
#include "serial_rw.h"
#include "sys_host.h"
#include "sys_mixer.h"

#include "../mod-controller-proto/mod-protocol.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// "sys_ver 07 version" -> "version"
#define SYS_CMD_ARG_START (_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2)

/*static*/ bool execute_ignoring_output(struct sp_port* const serialport, const char* argv[], const bool debug)
{
    if (execute(argv, debug))
    {
        if (debug)
            printf("%s(%p) completed successfully\n", __func__, argv);

        return serialport != NULL ? write_or_close(serialport, "r 0") : true;
    }

    if (debug)
        printf("%s(%p) failed\n", __func__, argv);

    return serialport != NULL ? write_or_close(serialport, "r -1") : false;
}

static bool execute_and_write_output_resp(struct sp_port* const serialport, const char* argv[], const bool debug)
{
    char cmdbuf[0xff + 4];

    if (execute_and_get_output(cmdbuf, argv, debug))
    {
        memmove(cmdbuf+4, cmdbuf, strlen(cmdbuf)+1);
        cmdbuf[0] = 'r';
        cmdbuf[1] = ' ';
        cmdbuf[2] = '0';
        cmdbuf[3] = ' ';

        if (debug)
            printf("%s(%p) completed successfully, responding with '%s'\n", __func__, argv, cmdbuf);

        return write_or_close(serialport, cmdbuf);
    }

    if (debug)
        printf("%s(%p) failed\n", __func__, argv);

    return write_or_close(serialport, "r -1");
}

static bool read_file_and_write_contents_resp(struct sp_port* const serialport, const char* filename, const bool debug)
{
    char cmdbuf[0xff + 4];

    if (read_file(cmdbuf, filename, debug))
    {
        memmove(cmdbuf+4, cmdbuf, strlen(cmdbuf)+1);
        cmdbuf[0] = 'r';
        cmdbuf[1] = ' ';
        cmdbuf[2] = '0';
        cmdbuf[3] = ' ';

        if (debug)
            printf("%s(%p) completed successfully, responding with '%s'\n", __func__, filename, cmdbuf);

        return write_or_close(serialport, cmdbuf);
    }

    if (debug)
        printf("%s(%p) failed\n", __func__, filename);

    return write_or_close(serialport, "r -1");
}

static bool write_int_resp(struct sp_port* const serialport, const int resp, const bool debug)
{
    char respbuf[0xff];
    snprintf(respbuf, sizeof(respbuf), "r 0 %i", resp);
    respbuf[sizeof(respbuf)-1] = '\0';

    if (debug)
        printf("sending response '%s'\n", respbuf);

    return write_or_close(serialport, respbuf);
}

static bool write_float_resp(struct sp_port* const serialport, const float resp, const bool debug)
{
    char respbuf[0xff];
    snprintf(respbuf, sizeof(respbuf), "r 0 %f", resp);
    respbuf[sizeof(respbuf)-1] = '\0';

    if (debug)
        printf("sending response '%s'\n", respbuf);

    return write_or_close(serialport, respbuf);
}

void create_postponed_messages_thread(const bool debug)
{
    sys_host_setup(debug);
    sys_mixer_setup(debug);
}

void process_postponed_messages(struct sp_port* const serialport)
{
    sys_host_process(serialport);
}

void destroy_postponed_messages_thread(void)
{
    sys_host_destroy();
    sys_mixer_destroy();
}

bool parse_and_reply_to_message(struct sp_port* const serialport, char msg[0xff], const bool debug)
{
    if (strncmp(msg, CMD_SYS_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* argvs = msg + SYS_CMD_ARG_START;

        // parsing arguments

        // io 0 = in, 1 = out
        const bool input = *argvs++ == '0';
        const char* const io = input ? "in" : "out";
        argvs++;

        // channel 1, 2 or 0 for both
        const char channel = *argvs++;
        const char channelstr[2] = { channel, '\0' };

        // mixer value (optional)
        const char* value = NULL;

        if (*argvs != '\0')
        {
            value = ++argvs;
            sys_mixer_gain(input, channel, value);
            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", io, channelstr, "xvol", value, NULL };

        return value == NULL ? execute_and_write_output_resp(serialport, argv, debug)
                             : execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_HP_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_mixer_headphone(value);
            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", "hp", "xvol", value, NULL };

        return value == NULL ? execute_and_write_output_resp(serialport, argv, debug)
                             : execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_CVI_MODE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_mixer_cv_exp_toggle(value);
            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", "cvexp", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_EXP_MODE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_mixer_exp_mode(value);
            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", "exppedal", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_CVO_MODE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_mixer_cv_headphone_toggle(value);
            return write_or_close(serialport, "r 0");
        }

        const char* argv[] = { "mod-amixer", "cvhp", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_AMIXER_SAVE, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-amixer", "save", NULL };

        return execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_BT_STATUS, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "hmi", NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_BT_DISCOVERY, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-bluetooth", "discovery", NULL };

        return execute_ignoring_output(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_SYSTEMCTL, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "systemctl", "is-active", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_VERSION, _CMD_SYS_LENGTH) == 0)
    {
        const char* argv[] = { "mod-version", msg + SYS_CMD_ARG_START, NULL };

        return execute_and_write_output_resp(serialport, argv, debug);
    }

    if (strncmp(msg, CMD_SYS_SERIAL, _CMD_SYS_LENGTH) == 0)
    {
        return read_file_and_write_contents_resp(serialport, "/var/cache/mod/tag", debug);
    }

    if (strncmp(msg, CMD_SYS_USB_MODE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        // changing to new mode
        if (value != NULL)
        {
            const char mode = *value;

            switch (mode)
            {
            case '0':
                delete_file("/data/enable-usb-multi-gadget", debug);
                delete_file("/data/enable-usb-windows-compat", debug);
                break;
            case '1':
                create_file("/data/enable-usb-multi-gadget", debug);
                delete_file("/data/enable-usb-windows-compat", debug);
                break;
            case '2':
                create_file("/data/enable-usb-multi-gadget", debug);
                create_file("/data/enable-usb-windows-compat", debug);
                break;
            }

            printf("%s: usb mode set to %c, sending 'r 0'\n", __func__, mode);
            return write_or_close(serialport, "r 0");
        }
        // reading current mode
        else
        {
            char mode = '0';

            if (access("/data/enable-usb-multi-gadget", F_OK) == 0)
            {
                if (access("/data/enable-usb-windows-compat", F_OK) == 0)
                    mode = '2';
                else
                    mode = '1';
            }

            char respbuf[6] = {
                'r', ' ', '0', ' ', mode, '\0'
            };

            printf("%s: usb mode request, sending '%s'\n", __func__, respbuf);
            return write_or_close(serialport, respbuf);
        }
    }

    if (strncmp(msg, CMD_SYS_NOISE_REMOVAL, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        // changing to new mode
        if (value != NULL)
        {
            const char mode = *value;

            switch (mode)
            {
            case '0':
                delete_file("/data/noise-removal-active", debug);
                break;
            case '1':
                create_file("/data/noise-removal-active", debug);
                break;
            }

            printf("%s: noise-removal mode set to %c, sending 'r 0'\n", __func__, mode);
            return write_or_close(serialport, "r 0");
        }
        // reading current mode
        else
        {
            char mode;

            if (access("/data/noise-removal-active", F_OK) == 0)
                mode = '1';
            else
                mode = '0';

            char respbuf[6] = {
                'r', ' ', '0', ' ', mode, '\0'
            };

            printf("%s: usb mode request, sending '%s'\n", __func__, respbuf);
            return write_or_close(serialport, respbuf);
        }
    }

    if (strncmp(msg, CMD_SYS_REBOOT, _CMD_SYS_LENGTH) == 0)
    {
        // HMI is useless after this point, so print resp asap and move on with the reboot
        write_or_close(serialport, "r 0");

        const char* argv_hmi_reset[] = { "hmi-reset", NULL };
        const char* argv_reboot[] = { "reboot", NULL };

        execute(argv_hmi_reset, debug);
        execute(argv_reboot, debug);
        return true;
    }

    if (strncmp(msg, CMD_SYS_COMP_MODE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const mode = strlen(msg) > SYS_CMD_ARG_START
                               ? msg + SYS_CMD_ARG_START
                               : NULL;

        if (mode != NULL)
        {
            sys_host_set_compressor_mode(atoi(mode));
            return write_or_close(serialport, "r 0");
        }

        return write_int_resp(serialport, sys_host_get_compressor_mode(), debug);
    }

    if (strncmp(msg, CMD_SYS_COMP_RELEASE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_host_set_compressor_release(atof(value));
            return write_or_close(serialport, "r 0");
        }

        return write_int_resp(serialport, sys_host_get_compressor_release(), debug);
    }

    if (strncmp(msg, CMD_SYS_NG_CHANNEL, _CMD_SYS_LENGTH) == 0)
    {
        const char* const channel = strlen(msg) > SYS_CMD_ARG_START
                                  ? msg + SYS_CMD_ARG_START
                                  : NULL;

        if (channel != NULL)
        {
            sys_host_set_noisegate_channel(atoi(channel));
            return write_or_close(serialport, "r 0");
        }

        return write_int_resp(serialport, sys_host_get_noisegate_channel(), debug);
    }

    if (strncmp(msg, CMD_SYS_NG_THRESHOLD, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_host_set_noisegate_threshold(atof(value));
            return write_or_close(serialport, "r 0");
        }

        return write_float_resp(serialport, sys_host_get_noisegate_threshold(), debug);
    }

    if (strncmp(msg, CMD_SYS_NG_DECAY, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_host_set_noisegate_decay(atof(value));
            return write_or_close(serialport, "r 0");
        }

        return write_int_resp(serialport, sys_host_get_noisegate_decay(), debug);
    }

    if (strncmp(msg, CMD_SYS_COMP_PEDALBOARD_GAIN, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value != NULL)
        {
            sys_host_set_pedalboard_gain(atof(value));
            return write_or_close(serialport, "r 0");
        }

        return write_float_resp(serialport, sys_host_get_pedalboard_gain(), debug);
    }

    if (strncmp(msg, CMD_SYS_PAGE_CHANGE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value == NULL)
            return write_or_close(serialport, "r -1");

        sys_host_set_hmi_page(atoi(value));
        return write_or_close(serialport, "r 0");
    }

    if (strncmp(msg, CMD_SYS_SUBPAGE_CHANGE, _CMD_SYS_LENGTH) == 0)
    {
        const char* const value = strlen(msg) > SYS_CMD_ARG_START
                                ? msg + SYS_CMD_ARG_START
                                : NULL;

        if (value == NULL)
            return write_or_close(serialport, "r -1");

        sys_host_set_hmi_subpage(atoi(value));
        return write_or_close(serialport, "r 0");
    }

    fprintf(stderr, "%s: unknown message '%s'\n", __func__, msg);
    return write_or_close(serialport, "r -1");
}
