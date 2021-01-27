/*
 * This file is part of mod-system-control.
 */

#include "cli.h"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>

bool execute(const char* argv[], const bool debug)
{
    if (debug)
    {
        printf("%s(%p) => \"%s", __func__, argv, argv[0]);
        for (int i=1; argv[i] != NULL; ++i)
            printf(" %s", argv[i]);
        printf("\"\n");
    }

    const pid_t pid = vfork();

    // error
    if (pid == -1)
        return false;

    // child process
    if (pid == 0)
    {
        close(STDIN_FILENO);

        execvp(argv[0], (char* const*)argv);

        fprintf(stderr, "cannot exec \"%s\": %s\n", argv[0], strerror(errno));
        _exit(EXIT_FAILURE);
        return false;
    }

    // wait for process to finish
    int state;
    if (waitpid(pid, &state, 0x0) < 0)
        return false;

    return true;
}

bool execute_and_get_output(char buf[0xff], const char* argv[], const bool debug)
{
    if (debug)
    {
        printf("%s(%p) => \"%s", __func__, argv, argv[0]);
        for (int i=1; argv[i] != NULL; ++i)
            printf(" %s", argv[i]);
        printf("\"\n");
    }

    int pipefd[2];
    pid_t pid;
    int state;
    ssize_t r;

    if (pipe(pipefd) != 0)
        goto error;

    pid = vfork();

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
        _exit(EXIT_FAILURE);
        return false;
    }

    // main process
    close(pipefd[1]);

    // wait for process to finish
    if (waitpid(pid, &state, 0x0) < 0)
        goto error;

    r = read(pipefd[0], buf, 0xff-1);

    if (r <= 0 || r >= 0xff)
        goto error;

    if (debug)
        printf("%s(%p) got %li bytes\n", __func__, argv, r);

    if (buf[r-1] == '\n')
        buf[r-1] = '\0';
    else
        buf[r] = '\0';

    return true;

error:
    memset(buf, 0, sizeof(char)*0xff);
    return false;
}

bool create_file(const char* const filename, const bool debug)
{
    if (access(filename, F_OK) == 0)
    {
        if (debug)
            printf("%s: filename '%s' already exists, doing nothing\n", __func__, filename);
        return true;
    }

    FILE* const fd = fopen(filename, "w");

    if (fd == NULL)
    {
        if (debug)
            printf("%s: failed to open filename '%s' for writing\n", __func__, filename);
        return true;

    }

    fwrite("", 1, 1, fd);
    fclose(fd);

    if (debug)
        printf("%s: filename '%s' was created successfully\n", __func__, filename);

    return true;
}

bool delete_file(const char* const filename, const bool debug)
{
    if (access(filename, F_OK) != 0)
    {
        if (debug)
            printf("%s: filename '%s' does not exist, doing nothing\n", __func__, filename);
        return true;
    }

    remove(filename);

    if (debug)
        printf("%s: filename '%s' was deleted successfully\n", __func__, filename);

    return true;
}

bool read_file(char buf[0xff], const char* const filename, const bool debug)
{
    FILE* const fd = fopen(filename, "r");

    if (fd == NULL)
    {
        if (debug)
            printf("%s: filename '%s' does not exist or failed to open\n", __func__, filename);
        memset(buf, 0, sizeof(char)*0xff);
        return false;
    }

    ssize_t r = fread(buf, 0xff-1, 1, fd);

    if (r < 0 || r >= 0xff)
        goto error;

    if (r == 0)
    {
        if (feof(fd) != 0)
            r = strlen(buf);
        else
            goto error;
    }

    if (debug)
        printf("%s: got %li bytes\n", __func__, r);

    if (buf[r-1] == '\n')
        buf[r-1] = '\0';
    else
        buf[r] = '\0';

    return true;

error:
    memset(buf, 0, sizeof(char)*0xff);
    return false;
}
