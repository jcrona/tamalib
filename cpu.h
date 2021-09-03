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
#ifndef _CPU_H_
#define _CPU_H_

#include "hal.h"

#define MEMORY_SIZE				4096 // 4096 x 4 bits (640 x 4 bits of RAM)

typedef struct breakpoint {
	u13_t addr;
	struct breakpoint *next;
} breakpoint_t;

/* Pins (TODO: add other pins) */
typedef enum {
	PIN_K00 = 0x0,
	PIN_K01 = 0x1,
	PIN_K02 = 0x2,
	PIN_K03 = 0x3,
	PIN_K10 = 0X4,
	PIN_K11 = 0X5,
	PIN_K12 = 0X6,
	PIN_K13 = 0X7,
} pin_t;

typedef enum {
	PIN_STATE_LOW = 0,
	PIN_STATE_HIGH = 1,
} pin_state_t;

typedef enum {
	INT_PROG_TIMER_SLOT = 0,
	INT_SERIAL_SLOT = 1,
	INT_K10_K13_SLOT = 2,
	INT_K00_K03_SLOT = 3,
	INT_STOPWATCH_SLOT = 4,
	INT_CLOCK_TIMER_SLOT = 5,
	INT_SLOT_NUM,
} int_slot_t;

typedef struct {
	u4_t factor_flag_reg;
	u4_t mask_reg;
	bool_t triggered; /* 1 if triggered, 0 otherwise */
	u8_t vector;
} interrupt_t;

typedef struct {
	u13_t *pc;
	u12_t *x;
	u12_t *y;
	u4_t *a;
	u4_t *b;
	u5_t *np;
	u8_t *sp;
	u4_t *flags;

	u32_t *tick_counter;
	u32_t *clk_timer_timestamp;
	u32_t *prog_timer_timestamp;
	bool_t *prog_timer_enabled;
	u8_t *prog_timer_data;
	u8_t *prog_timer_rld;

	u32_t *call_depth;

	interrupt_t *interrupts;

	u4_t *memory;
} state_t;


void cpu_add_bp(breakpoint_t **list, u13_t addr);
void cpu_free_bp(breakpoint_t **list);

void cpu_set_speed(u8_t speed);

state_t * cpu_get_state(void);

u32_t cpu_get_depth(void);

void cpu_set_input_pin(pin_t pin, pin_state_t state);

void cpu_sync_ref_timestamp(void);

void cpu_refresh_hw(void);

bool_t cpu_init(const u12_t *program, breakpoint_t *breakpoints, u32_t freq);
void cpu_release(void);

int cpu_step(void);

#endif /* _CPU_H_ */
