/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#include <stdbool.h>

void sys_host_setup(bool debug);
void sys_host_process(struct sp_port* serialport);
void sys_host_destroy(void);

int sys_host_get_compressor_mode(void);
int sys_host_get_compressor_release(void);
int sys_host_get_noisegate_channel(void);
int sys_host_get_noisegate_decay(void);
int sys_host_get_noisegate_threshold(void);
int sys_host_get_pedalboard_gain(void);

void sys_host_set_compressor_mode(int mode);
void sys_host_set_compressor_release(int value);
void sys_host_set_noisegate_channel(int channel);
void sys_host_set_noisegate_decay(int value);
void sys_host_set_noisegate_threshold(int value);
void sys_host_set_pedalboard_gain(int value);

void sys_host_set_hmi_page(int page);
void sys_host_set_hmi_subpage(int subpage);
