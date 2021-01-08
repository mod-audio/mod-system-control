/*
 * This file is part of mod-system-control.
 */

#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <unistd.h>

#include <libserialport.h>
#include <sys/stat.h>
#include <systemd/sd-daemon.h>

// TODO put this in spec file
#define MOD_SYSTEM_SERIAL_CMD_PREFIX "sys_"
#define MOD_SYSTEM_SERIAL_CMD_SIZE  7 /* "cmd_xyz" */
#define MOD_SYSTEM_SERIAL_DATA_SIZE 2 /* "ff" max */

#define MOD_SYSTEM_SERIAL_CMD_VERSION MOD_SYSTEM_SERIAL_CMD_PREFIX "ver"
#define MOD_SYSTEM_SERIAL_CMD_BT_INFO MOD_SYSTEM_SERIAL_CMD_PREFIX "bti"
#define MOD_SYSTEM_SERIAL_CMD_BT_DISC MOD_SYSTEM_SERIAL_CMD_PREFIX "btd"

static volatile bool g_running = true;

static void signal_handler(int sig)
{
    g_running = false;
    return;

    // unused
    (void)sig;
}

static struct sp_port* serial_open(const char* const serial, const int baudrate)
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
        goto error_free;
    }

    // disable XON/XOFF flow control
    sp_set_xon_xoff(serialport, SP_XONXOFF_DISABLED);

    // configure serial port
    sp_set_baudrate(serialport, baudrate);

    // cleanup
    if (resolvedserial != serial)
        free((char*)resolvedserial);

    return serialport;

error_free:
    if (resolvedserial != serial)
        free((char*)resolvedserial);

    return NULL;
}

static void serial_close(struct sp_port* const serialport)
{
    sp_close(serialport);
    sp_free_port(serialport);
}

static uint8_t serial_read_until_zero(struct sp_port* const serialport, char buf[0xff])
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

static bool write_or_close(struct sp_port* serialport, const char* const resp)
{
    errno = 0;
    if (sp_nonblocking_write(serialport, resp, strlen(resp)+1) == SP_ERR_FAIL && errno == EIO)
        return false;
    return true;
}

static bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff])
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

int main(int argc, char* argv[])
{
    struct sp_port* serialport;
    const char* serial;
    int baudrate;
    char buf[0xff];
    uint8_t msg_size;

    if (argc <= 2)
    {
        fprintf(stdout, "Usage: %s <serial-device> <speed>\n", argv[0]);
        return EXIT_FAILURE;
    }

    // parse arguments
    serial = argv[1];
    baudrate = atoi(argv[2]);

    // open serial port
    serialport = serial_open(serial, baudrate);

    if (serialport == NULL)
        return EXIT_FAILURE;

    // setup quit signal
    struct sigaction sig;
    memset(&sig, 0, sizeof(sig));

    sig.sa_handler = signal_handler;
    sig.sa_flags   = SA_RESTART;
    sigemptyset(&sig.sa_mask);
    sigaction(SIGTERM, &sig, NULL);
    sigaction(SIGINT, &sig, NULL);

    // notify systemd we are running
    fprintf(stdout, "%s now running with '%s' and %d as parameters\n", argv[0], serial, baudrate);
    sd_notify(0, "READY=1");

    while (g_running)
    {
        if ((msg_size = serial_read_until_zero(serialport, buf)) == 0)
            continue;

        if (! parse_and_reply_to_message(serialport, buf))
            break;
    }

    // notify systemd we are stopping
    sd_notify(0, "STOPPING=1");
    fprintf(stdout, "%s stopping...\n", argv[0]);

    // close serial port
    serial_close(serialport);

    return EXIT_SUCCESS;
}
