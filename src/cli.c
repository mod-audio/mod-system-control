/*
 * This file is part of mod-system-control.
 */

#include "cli.h"

#define _GNU_SOURCE
#include <string.h>

bool execute_and_get_output(char buf[0xff], char* argv[])
{
    memset(buf, 0, sizeof(char)*0xff);
    return false;

    // unused
    (void)argv;
}
