/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#include <stdbool.h>

// calls write_or_close as final step
// if this function returns false, serial is no longer valid
bool parse_and_reply_to_message(struct sp_port* serialport, char msg[0xff], bool debug);

void create_postponed_messages_thread(bool debug);
void destroy_postponded_messages_thread(void);
