/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

struct sp_port* serial_open(const char* serial, int baudrate);
void serial_close(struct sp_port* serialport);
