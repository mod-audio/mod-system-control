/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#define _GNU_SOURCE
#include <stdbool.h>
#include <stdint.h>

typedef enum sp_read_error_status {
    /* there was nothing to read, try again */
    SP_READ_ERROR_NO_DATA = -1,
    /* we read something, but data was invalid. call serial_read_ignore_until_zero next */
    SP_READ_ERROR_INVALID_DATA = -2,
    /* IO error while reading, likely due to serial device being disconnected */
    SP_READ_ERROR_IO = -3,
    /* unknown read error */
    SP_READ_ERROR_UNKNOWN = -4,
} sp_read_error_status;

sp_read_error_status serial_read_msg_until_zero(struct sp_port* serialport, char buf[0xff]);
sp_read_error_status serial_read_ignore_until_zero(struct sp_port* serialport);
bool write_or_close(struct sp_port* serialport, const char* resp);
