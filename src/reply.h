/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#include <stdbool.h>

bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff]);
