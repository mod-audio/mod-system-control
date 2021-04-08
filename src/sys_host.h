/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#include <stdbool.h>

void sys_host_setup(bool debug);
void sys_host_process(struct sp_port* serialport);
void sys_host_destroy();
