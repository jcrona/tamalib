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
#include "tamalib.h"
#include "hw.h"
#include "cpu.h"
#include "hal.h"

#define DEFAULT_FRAMERATE				30 // fps

static bool_t step_by_step_en = 0;
static bool_t is_paused = 0;

static timestamp_t screen_ts = 0;

static u32_t ts_freq;

static u8_t g_framerate = DEFAULT_FRAMERATE;

hal_t *g_hal;


bool_t tamalib_init(const u12_t *program, breakpoint_t *breakpoints, u32_t freq)
{
	bool_t res = 0;

	res |= cpu_init(program, breakpoints, freq);
	res |= hw_init();

	ts_freq = freq;

	return res;
}

void tamalib_release(void)
{
	hw_release();
	cpu_release();
}

void tamalib_set_framerate(u8_t framerate)
{
	g_framerate = framerate;
}

u8_t tamalib_get_framerate(void)
{
	return g_framerate;
}

void tamalib_register_hal(hal_t *hal)
{
	g_hal = hal;
}

void tamalib_pause(bool_t en)
{
	is_paused = en;
}

void tamalib_enable_step_by_step(bool_t en)
{
	step_by_step_en = en;
}

void tamalib_step(void)
{
	if (is_paused) {
		return;
	}

	is_paused = cpu_step();

	if (step_by_step_en) {
		is_paused = 1;
	}
}

void tamalib_mainloop(void)
{
	timestamp_t ts;

	while (!g_hal->handler()) {
		tamalib_step();

		/* Update the screen @ g_framerate fps */
		ts = g_hal->get_timestamp();
		if (ts - screen_ts >= ts_freq/g_framerate) {
			screen_ts = ts;
			g_hal->update_screen();
		}
	}
}
