/*
 * This file is part of mod-system-control.
 */

#pragma once

#include <stdbool.h>

bool execute(const char* argv[], bool debug);
bool execute_and_get_output(char buf[0xff], const char* argv[], bool debug);

bool create_file(const char* filename, bool debug);
bool delete_file(const char* filename, bool debug);
bool read_file(char buf[0xff], const char* filename, bool debug);
bool write_file(char buf[0xff], const char* filename, bool debug);
