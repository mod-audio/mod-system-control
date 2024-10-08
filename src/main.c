/*
 * This file is part of mod-system-control.
 */

#include "serial_io.h"
#include "serial_rw.h"
#include "reply.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <signal.h>

#ifdef HAVE_SYSTEMD
#include <systemd/sd-daemon.h>
#endif

static volatile bool g_running = true;

static void signal_handler(int sig)
{
    g_running = false;
    return;

    // unused
    (void)sig;
}

int main(int argc, char* argv[])
{
    struct sp_port* serialport;
    const char* serial;
    int baudrate;
    char buf[0xff];

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

    // flush buffers
    sp_flush(serialport, SP_BUF_BOTH);

    // check if debugging
    const char* const mod_log = getenv("MOD_LOG");
    const bool debug = atoi(mod_log != NULL ? mod_log : "0");

    // setup quit signal
    struct sigaction sig;
    memset(&sig, 0, sizeof(sig));

    sig.sa_handler = signal_handler;
    sig.sa_flags   = SA_RESTART;
    sigemptyset(&sig.sa_mask);
    sigaction(SIGTERM, &sig, NULL);
    sigaction(SIGINT, &sig, NULL);

    // create thread for postponed messages
    create_postponed_messages_thread(debug);

    // notify we are running
    fprintf(stdout, "%s now running with '%s', %d baudrate and logging %s\n",
            argv[0], serial, baudrate, debug ? "enabled" : "disabled");
#ifdef HAVE_SYSTEMD
    sd_notify(0, "READY=1");
#endif

    while (g_running)
    {
        switch (serial_read_msg_until_zero(serialport, buf, debug))
        {
        case SP_READ_ERROR_NO_DATA:
            process_postponed_messages(serialport);
            continue;
        case SP_READ_ERROR_INVALID_DATA:
            serial_read_ignore_until_zero(serialport);
            process_postponed_messages(serialport);
            continue;
        case SP_READ_ERROR_IO:
            break;
        }

        if (debug)
        {
            fprintf(stdout, "mod-system-control received '%s'\n", buf);
            fflush(stdout);
        }

        if (! parse_and_reply_to_message(serialport, buf, debug))
            break;
    }

    // notify we are stopping
#ifdef HAVE_SYSTEMD
    sd_notify(0, "STOPPING=1");
#endif
    fprintf(stdout, "%s stopping...\n", argv[0]);

    // close postponed messages thread
    destroy_postponed_messages_thread();

    // close serial port
    serial_close(serialport);

    return EXIT_SUCCESS;
}
