/*
 * TamaLIB - A hardware agnostic Tamagotchi P1 emulation library
 *
 * Copyright (C) 2021 Jean-Christophe Rona <jc@rona.fr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef _TAMALIB_H_
#define _TAMALIB_H_

#include "cpu.h"
#include "hw.h"
#include "hal.h"

#define FRAMERATE					30 // fps

#define tamalib_set_button(btn, state)			hw_set_button(btn, state)

#define tamalib_set_speed(speed)			cpu_set_speed(speed)

#define tamalib_get_state()				cpu_get_state()

#define tamalib_add_bp(list, addr)			cpu_add_bp(list, addr)
#define tamalib_free_bp(list)				cpu_free_bp(list)


void tamalib_release(void);
bool_t tamalib_init(u12_t *program, breakpoint_t *breakpoints);

void tamalib_register_hal(hal_t *hal);

void tamalib_pause(bool_t en);
void tamalib_enable_step_by_step(bool_t en);

/* NOTE: Only one of these two functions must be used in the main application
 * (tamalib_step() should be used only if tamalib_mainloop() does not fit the
 * main application execution flow).
 */
void tamalib_step(void);
void tamalib_mainloop(void);

#endif /* _TAMALIB_H_ */
