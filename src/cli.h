/*
 * This file is part of mod-system-control.
 */

#pragma once

#include <stdbool.h>

bool execute(const char* argv[], bool debug);
bool execute_and_get_output(char buf[0xff], const char* argv[], bool debug);
