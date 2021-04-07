/*
 * This file is part of mod-system-control.
 */

#pragma once

#define SYS_SERIAL_SHM "/sys_msgs"

// #include "lv2-hmi.h"

#include "mod-semaphore.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

// must be 8192 - sizeof sys_serial_shm_data members, so we cleanly align to 64bits
#define SYS_SERIAL_SHM_DATA_SIZE (8192 - sizeof(sem_t) - sizeof(uint32_t)*2)

// FIXME should we use binary form, or string?
// FIXME using string with "%c %s" syntax? (for 'l' led, 'n' name, 'v' value, 'u' unit)
typedef enum {
    sys_serial_event_type_null = 0,
    sys_serial_event_type_led,
    sys_serial_event_type_label,
    sys_serial_event_type_unit
} sys_serial_event_type;

typedef struct {
    sys_serial_event_type type;
    char value[8];
} sys_serial_event;

typedef struct {
    // semaphore for syncing
    sem_t sem;
    // for ring-buffer access
    uint32_t head, tail;
    // actual buffer
    uint8_t buffer[SYS_SERIAL_SHM_DATA_SIZE];
} sys_serial_shm_data;

static inline
bool sys_serial_open(int* shmfd, sys_serial_shm_data** data)
{
    int fd;
    sem_t sem;
    sys_serial_shm_data* ptr;

#ifdef SERVER_MODE
    fd = shm_open(SYS_SERIAL_SHM, O_CREAT|O_EXCL|O_RDWR, 0600);
#else
    fd = shm_open(SYS_SERIAL_SHM, O_RDWR, 0);
#endif

    if (fd < 0)
    {
        fprintf(stderr, "shm_open failed\n");
        return false;
    }

#ifdef SERVER_MODE
    if (ftruncate(fd, sizeof(sys_serial_shm_data)) != 0)
    {
        fprintf(stderr, "ftruncate failed\n");
        goto cleanup;
    }
    if (sem_init(&sem, 1, 0) != 0)
    {
        fprintf(stderr, "sem_init failed\n");
        goto cleanup;
    }
#endif

    ptr = (sys_serial_shm_data*)mmap(NULL,
                                     sizeof(sys_serial_shm_data),
                                     PROT_READ|PROT_WRITE,
                                     MAP_SHARED|MAP_LOCKED,
                                     fd,
                                     0);

    if (ptr == NULL || ptr == MAP_FAILED)
    {
        fprintf(stderr, "mmap failed\n");
        goto cleanup_sem;
    }

    memset(ptr, 0, sizeof(sys_serial_shm_data));
    ptr->sem = sem;

    *shmfd = fd;
    *data = ptr;
    return true;

cleanup_sem:
#ifdef SERVER_MODE
    sem_destroy(&sem);
#endif

cleanup:
    close(fd);
#ifdef SERVER_MODE
    shm_unlink(SYS_SERIAL_SHM);
#endif

    *shmfd = 0;
    *data = NULL;
    return false;
}

static inline
void sys_serial_close(int shmfd, sys_serial_shm_data* data)
{
#ifdef SERVER_MODE
    sem_destroy(&data->sem);
#endif
    munmap(data, sizeof(sys_serial_shm_data));

    close(shmfd);
#ifdef SERVER_MODE
    shm_unlink(SYS_SERIAL_SHM);
#endif
}

// server
static inline
bool sys_serial_read(sys_serial_shm_data* data, sys_serial_event* event)
{
    if (data->head == data->tail)
        return false;

    // TODO
    event->type = sys_serial_event_type_null;
    return true;
}

// client
static inline
bool sys_serial_write(sys_serial_shm_data* data, const uint8_t* buf, uint32_t size)
{
    if (size == 0)
        return false;
    if (size >= SYS_SERIAL_SHM_DATA_SIZE)
        return false;

    const uint32_t head = data->head;
    const uint32_t tail = data->tail;
    const uint32_t wrap = tail > head ? 0 : SYS_SERIAL_SHM_DATA_SIZE;

    if (size >= wrap + tail - head)
    {
        fprintf(stderr, "sys_serial_write: failed, not enough space\n");
        return false;
    }

    uint32_t nexthead = head + size;

    if (nexthead > SYS_SERIAL_SHM_DATA_SIZE)
    {
        nexthead -= SYS_SERIAL_SHM_DATA_SIZE;

        if (size == 1)
        {
            memcpy(data->buffer + head, buf, size);
        }
        else
        {
            const uint32_t firstpart = SYS_SERIAL_SHM_DATA_SIZE - head;
            memcpy(data->buffer + head, buf, firstpart);
            memcpy(data->buffer, buf + firstpart, nexthead);
        }
    }
    else
    {
        memcpy(data->buffer + head, buf, size);

        if (nexthead == SYS_SERIAL_SHM_DATA_SIZE)
            nexthead = 0;
    }

    data->head = nexthead;
    sem_post(&data->sem);
    return true;
}
