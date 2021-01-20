/*
 * This file is part of mod-system-control.
 */

#include "cli.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

bool execute_and_get_output(char buf[0xff], const char* argv[])
{
    /*
    printf("%s(%p) => \"%s", __func__, argv, argv[0]);
    for (int i=1; argv[i] != NULL; ++i)
        printf(" %s", argv[i]);
    printf("\"\n");
    */

    int pipefd[2];

    if (pipe(pipefd) != 0)
        goto error;

    const pid_t pid = vfork();

    // error
    if (pid == -1)
        goto error;

    // child process
    if (pid == 0)
    {
        fcntl(pipefd[0], F_SETFD, FD_CLOEXEC);

        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[1]);
        close(pipefd[0]);
        close(STDIN_FILENO);

        execvp(argv[0], (char* const*)argv);

        fprintf(stderr, "cannot exec \"%s\": %s\n", argv[0], strerror(errno));
        _exit(1);
        return false;
    }

    // main process
    close(pipefd[1]);

    // wait for process to finish
    int state;
    if (waitpid(pid, &state, 0x0) < 0)
        goto error;

    const ssize_t r = read(pipefd[0], buf, 0xff-1);

    if (r <= 0 || r >= 0xff)
        goto error;

    /*
    printf("%s(%p) got %li bytes\n", __func__, argv, r);
    */

    if (buf[r-1] == '\n')
        buf[r-1] = '\0';
    else
        buf[r] = '\0';

    return true;

error:
    memset(buf, 0, sizeof(char)*0xff);
    return false;
}
