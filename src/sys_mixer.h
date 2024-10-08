/*
 * This file is part of mod-system-control.
 */

#pragma once

#include <stdbool.h>

void sys_mixer_setup(bool debug);
void sys_mixer_destroy();

void sys_mixer_gain(bool input, char channel, const char* value);
void sys_mixer_headphone(const char* value);
void sys_mixer_cv_exp_toggle(const char* value);
void sys_mixer_exp_mode(const char* value);
void sys_mixer_cv_headphone_toggle(const char* value);