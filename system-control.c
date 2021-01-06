/*
 * This file is part of mod-system-control.
 */

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
    if (argc <= 1)
    {
        fprintf(stdout, "Usage: %s <serial-device>\n", argv[0]);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
