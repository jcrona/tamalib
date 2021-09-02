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
#include "cpu.h"
#include "hw.h"
#include "hal.h"

#define TICK_FREQUENCY				32768 // Hz

#define TIMER_1HZ_PERIOD			32768 // in ticks
#define TIMER_256HZ_PERIOD			128 // in ticks

#define MASK_4B					0xF00
#define MASK_6B					0xFC0
#define MASK_7B					0xFE0
#define MASK_8B					0xFF0
#define MASK_10B				0xFFC
#define MASK_12B				0xFFF

#define PCS					(pc & 0xFF)
#define PCSL					(pc & 0xF)
#define PCSH					((pc >> 4) & 0xF)
#define PCP					((pc >> 8) & 0xF)
#define PCB					((pc >> 12) & 0x1)
#define TO_PC(bank, page, step)			((step & 0xFF) | ((page & 0xF) << 8) | (bank & 0x1) << 12)
#define NBP					((np >> 4) & 0x1)
#define NPP					(np & 0xF)
#define TO_NP(bank, page)			((page & 0xF) | (bank & 0x1) << 4)
#define XHL					(x & 0xFF)
#define XL					(x & 0xF)
#define XH					((x >> 4) & 0xF)
#define XP					((x >> 8) & 0xF)
#define YHL					(y & 0xFF)
#define YL					(y & 0xF)
#define YH					((y >> 4) & 0xF)
#define YP					((y >> 8) & 0xF)
#define M(n)					get_memory(n)
#define SET_M(n, v)				set_memory(n, v)
#define RQ(i)					get_rq(i)
#define SET_RQ(i, v)				set_rq(i, v)
#define SPL					(sp & 0xF)
#define SPH					((sp >> 4) & 0xF)

#define FLAG_C					(0x1 << 0)
#define FLAG_Z					(0x1 << 1)
#define FLAG_D					(0x1 << 2)
#define FLAG_I					(0x1 << 3)

#define C					!!(flags & FLAG_C)
#define Z					!!(flags & FLAG_Z)
#define D					!!(flags & FLAG_D)
#define I					!!(flags & FLAG_I)

#define SET_C()					{flags |= FLAG_C;}
#define CLEAR_C()				{flags &= ~FLAG_C;}
#define SET_Z()					{flags |= FLAG_Z;}
#define CLEAR_Z()				{flags &= ~FLAG_Z;}
#define SET_D()					{flags |= FLAG_D;}
#define CLEAR_D()				{flags &= ~FLAG_D;}
#define SET_I()					{flags |= FLAG_I;}
#define CLEAR_I()				{flags &= ~FLAG_I;}

#define INPUT_PORT_NUM				2

typedef struct {
	char *log;
	u12_t code;
	u12_t mask;
	u12_t shift_arg0;
	u12_t mask_arg0;			// != 0 only if there are two arguments
	u8_t cycles;
	void (*cb)(u8_t arg0, u8_t arg1);
} op_t;

typedef struct {
	u4_t states;
} input_port_t;

/* Registers */
static u13_t pc, next_pc;
static u12_t x, y;
static u4_t a, b;
static u5_t np;
static u8_t sp;

/* Flags */
static u4_t flags;

#define PROGRAM_SIZE 6144
static const u12_t *g_program = NULL;
static u4_t memory[MEMORY_SIZE];

static input_port_t inputs[INPUT_PORT_NUM] = {{0}};

/* Interrupts (in priority order) */
static interrupt_t interrupts[INT_SLOT_NUM] = {
	{0x0, 0x0, 0, 0x0C}, // Prog timer
	{0x0, 0x0, 0, 0x0A}, // Serial interface
	{0x0, 0x0, 0, 0x08}, // Input (K10-K13)
	{0x0, 0x0, 0, 0x06}, // Input (K00-K03)
	{0x0, 0x0, 0, 0x04}, // Stopwatch timer
	{0x0, 0x0, 0, 0x02}, // Clock timer
};
static bool_t interrupt_asserted = 0;

static breakpoint_t *g_breakpoints = NULL;

static u32_t call_depth = 0;

static u32_t clk_timer_timestamp = 0; // in ticks
static u32_t prog_timer_timestamp = 0; // in ticks
static bool_t prog_timer_enabled = 0;
static u8_t prog_timer_data = 0;
static u8_t prog_timer_rld = 0;

static u32_t tick_counter = 0;
static u32_t ts_freq;
static u8_t speed_ratio = 1;
static timestamp_t ref_ts;

static state_t cpu_state = {
	.pc = &pc,
	.x = &x,
	.y = &y,
	.a = &a,
	.b = &b,
	.np = &np,
	.sp = &sp,
	.flags = &flags,

	.tick_counter = &tick_counter,
	.clk_timer_timestamp = &clk_timer_timestamp,
	.prog_timer_timestamp = &prog_timer_timestamp,
	.prog_timer_enabled = &prog_timer_enabled,
	.prog_timer_data = &prog_timer_data,
	.prog_timer_rld = &prog_timer_rld,

	.call_depth = &call_depth,

	.interrupts = interrupts,

	.memory = memory,
};


void cpu_add_bp(breakpoint_t **list, u13_t addr)
{
	breakpoint_t *bp;

	bp = (breakpoint_t *) g_hal->malloc(sizeof(breakpoint_t));
	if (!bp) {
		g_hal->log(LOG_ERROR, "Cannot allocate memory for breakpoint 0x%04X!\n", addr);
		return;
	}

	bp->addr = addr;

	if (*list != NULL) {
		bp->next = *list;
	} else {
		/* List is empty */
		bp->next = NULL;
	}

	*list = bp;
}

void cpu_free_bp(breakpoint_t **list)
{
	breakpoint_t *bp = *list, *tmp;

	while (bp != NULL) {
		tmp = bp->next;
		g_hal->free(bp);
		bp = tmp;
	}

	*list = NULL;
}

void cpu_set_speed(u8_t speed)
{
	speed_ratio = speed;
}

state_t * cpu_get_state(void)
{
	return &cpu_state;
}

u32_t cpu_get_depth(void)
{
	return call_depth;
}

static void generate_interrupt(int_slot_t slot, u8_t bit)
{
	/* Set the factor flag no matter what */
	interrupts[slot].factor_flag_reg = interrupts[slot].factor_flag_reg | (0x1 << bit);

	/* Trigger the INT only if not masked */
	if (interrupts[slot].mask_reg & (0x1 << bit)) {
		interrupts[slot].triggered = 1;
		interrupt_asserted = 1;
	}
}

void cpu_set_input_pin(pin_t pin, pin_state_t state)
{
	/* Set the I/O */
	inputs[pin & 0x4].states = (inputs[pin & 0x4].states & ~(0x1 << (pin & 0x3))) | (state << (pin & 0x3));

	/* Trigger the interrupt (TODO: handle relation register) */
	if (state == PIN_STATE_LOW) {
		switch ((pin & 0x4) >> 2) {
			case 0:
				generate_interrupt(INT_K00_K03_SLOT, pin & 0x3);
				break;

			case 1:
				generate_interrupt(INT_K10_K13_SLOT, pin & 0x3);
				break;
		}
	}
}

void cpu_sync_ref_timestamp(void)
{
	ref_ts = g_hal->get_timestamp();
}

static u4_t get_io(u12_t n)
{
	u4_t tmp;

	switch (n) {
		case 0xF00:
			/* Interrupt factor flags (clock timer) */
			tmp = interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg;
			interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF01:
			/* Interrupt factor flags (stopwatch) */
			tmp = interrupts[INT_STOPWATCH_SLOT].factor_flag_reg;
			interrupts[INT_STOPWATCH_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF02:
			/* Interrupt factor flags (prog timer) */
			tmp = interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg;
			interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF03:
			/* Interrupt factor flags (serial) */
			tmp = interrupts[INT_SERIAL_SLOT].factor_flag_reg;
			interrupts[INT_SERIAL_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF04:
			/* Interrupt factor flags (K00-K03) */
			tmp = interrupts[INT_K00_K03_SLOT].factor_flag_reg;
			interrupts[INT_K00_K03_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF05:
			/* Interrupt factor flags (K10-K13) */
			tmp = interrupts[INT_K10_K13_SLOT].factor_flag_reg;
			interrupts[INT_K10_K13_SLOT].factor_flag_reg = 0;
			return tmp;

		case 0xF10:
			/* Clock timer interrupt masks */
			return interrupts[INT_CLOCK_TIMER_SLOT].mask_reg;

		case 0xF11:
			/* Stopwatch interrupt masks */
			return interrupts[INT_STOPWATCH_SLOT].mask_reg & 0x3;

		case 0xF12:
			/* Prog timer interrupt masks */
			return interrupts[INT_PROG_TIMER_SLOT].mask_reg & 0x1;

		case 0xF13:
			/* Serial interface interrupt masks */
			return interrupts[INT_SERIAL_SLOT].mask_reg & 0x1;

		case 0xF14:
			/* Input (K00-K03) interrupt masks */
			return interrupts[INT_K00_K03_SLOT].mask_reg;

		case 0xF15:
			/* Input (K10-K13) interrupt masks */
			return interrupts[INT_K10_K13_SLOT].mask_reg;

		case 0xF24:
			/* Prog timer data (low) */
			return prog_timer_data & 0xF;

		case 0xF25:
			/* Prog timer data (high) */
			return (prog_timer_data >> 4) & 0xF;

		case 0xF26:
			/* Prog timer reload data (low) */
			return prog_timer_rld & 0xF;

		case 0xF27:
			/* Prog timer reload data (high) */
			return (prog_timer_rld >> 4) & 0xF;

		case 0xF40:
			/* Input port (K00-K03) */
			return inputs[0].states;

		case 0xF42:
			/* Input port (K10-K13) */
			return inputs[1].states;

		case 0xF54:
			/* Output port (R40-R43) */
			return memory[n];

		case 0xF70:
			/* CPU/OSC3 clocks switch, CPU voltage switch */
			return memory[n];

		case 0xF71:
			/* LCD control */
			return memory[n];

		case 0xF72:
			/* LCD contrast */
			break;

		case 0xF73:
			/* SVD */
			return memory[n] & 0x7; // Voltage always OK

		case 0xF74:
			/* Buzzer config 1 */
			return memory[n];

		case 0xF75:
			/* Buzzer config 2 */
			return memory[n] & 0x3; // Buzzer ready

		case 0xF76:
			/* Clock/Watchdog timer reset */
			break;

		case 0xF77:
			/* Stopwatch stop/run/reset */
			break;

		case 0xF78:
			/* Prog timer stop/run/reset */
			return !!prog_timer_enabled;

		case 0xF79:
			/* Prog timer clock selection */
			break;

		default:
			g_hal->log(LOG_ERROR,   "Read from unimplemented I/O 0x%03X - PC = 0x%04X\n", n, pc);
	}

	return 0;
}

static void set_io(u12_t n, u4_t v)
{
	switch (n) {
		// Note: "Be sure that writing to the interrupt mask register is
		// done with the interrupt in the disable state." So clearing the
		// mask register *shouldn't* trigger the interrupt.
		case 0xF10:
			/* Clock timer interrupt masks */
			/* Assume 1Hz timer INT enabled (0x8) */
			interrupts[INT_CLOCK_TIMER_SLOT].mask_reg = v;
			break;

		case 0xF11:
			/* Stopwatch interrupt masks */
			/* Assume all INT disabled */
			interrupts[INT_STOPWATCH_SLOT].mask_reg = v;
			break;

		case 0xF12:
			/* Prog timer interrupt masks */
			/* Assume Prog timer INT enabled (0x1) */
			interrupts[INT_PROG_TIMER_SLOT].mask_reg = v;
			break;

		case 0xF13:
			/* Serial interface interrupt masks */
			/* Asumme all INT disabled */
			interrupts[INT_K10_K13_SLOT].mask_reg = v;
			break;

		case 0xF14:
			/* Input (K00-K03) interrupt masks */
			/* Asumme all INT disabled */
			interrupts[INT_SERIAL_SLOT].mask_reg = v;
			break;

		case 0xF15:
			/* Input (K10-K13) interrupt masks */
			/* Asumme all INT disabled */
			interrupts[INT_K10_K13_SLOT].mask_reg = v;
			break;

		case 0xF26:
			/* Prog timer reload data (low) */
			prog_timer_rld = v | (prog_timer_rld & 0xF0);
			break;

		case 0xF27:
			/* Prog timer reload data (high) */
			prog_timer_rld = (prog_timer_rld & 0xF) | (v << 4);
			break;

		case 0xF40:
			/* Input port (K00-K03) */
			/* Write not allowed */
			break;

		case 0xF54:
			/* Output port (R40-R43) */
			//g_hal->log(LOG_INFO, "Output/Buzzer: 0x%X\n", v);
			hw_enable_buzzer(!(v & 0x8));
			break;

		case 0xF70:
			/* CPU/OSC3 clocks switch, CPU voltage switch */
			/* Assume 32,768 OSC1 selected, OSC3 off, battery >= 3,1V (0x1) */
			break;

		case 0xF71:
			/* LCD control */
			break;

		case 0xF72:
			/* LCD contrast */
			/* Assume medium contrast (0x8) */
			break;

		case 0xF73:
			/* SVD */
			/* Assume battery voltage always OK (0x6) */
			break;

		case 0xF74:
			/* Buzzer config 1 */
			hw_set_buzzer_freq(v & 0x7);
			break;

		case 0xF75:
			/* Buzzer config 2 */
			break;

		case 0xF76:
			/* Clock/Watchdog timer reset */
			/* Ignore watchdog */
			break;

		case 0xF77:
			/* Stopwatch stop/run/reset */
			break;

		case 0xF78:
			/* Prog timer stop/run/reset */
			if (v & 0x2) {
				prog_timer_data = prog_timer_rld;
			}

			if ((v & 0x1) && !prog_timer_enabled) {
				prog_timer_timestamp = tick_counter;
			}

			prog_timer_enabled = v & 0x1;
			break;

		case 0xF79:
			/* Prog timer clock selection */
			/* Assume 256Hz, output disabled */
			break;

		default:
			g_hal->log(LOG_ERROR,   "Write 0x%X to unimplemented I/O 0x%03X - PC = 0x%04X\n", v, n, pc);
	}
}

static void set_lcd(u12_t n, u4_t v)
{
	u8_t i;
	u8_t seg, com0;

	seg = ((n & 0x7F) >> 1);
	com0 = (((n & 0x80) >> 7) * 8 + (n & 0x1) * 4);

	for (i = 0; i < 4; i++) {
		hw_set_lcd_pin(seg, com0 + i, (v >> i) & 0x1);
	}
}

static u4_t get_memory(u12_t n)
{
	u4_t res = 0;

	if (n < 0x280) {
		/* RAM */
		g_hal->log(LOG_MEMORY, "RAM              - ");
		res = memory[n];
	} else if (n >= 0xE00 && n < 0xE50) {
		/* Display Memory 1 */
		g_hal->log(LOG_MEMORY, "Display Memory 1 - ");
		res = memory[n];
	} else if (n >= 0xE80 && n < 0xED0) {
		/* Display Memory 2 */
		g_hal->log(LOG_MEMORY, "Display Memory 2 - ");
		res = memory[n];
	} else if (n >= 0xF00 && n < 0xF80) {
		/* I/O Memory */
		g_hal->log(LOG_MEMORY, "I/O              - ");
		res = get_io(n);
	} else {
		g_hal->log(LOG_ERROR,   "Read from invalid memory address 0x%03X - PC = 0x%04X\n", n, pc);
		return 0;
	}

	g_hal->log(LOG_MEMORY, "Read  0x%X - Address 0x%03X - PC = 0x%04X\n", res, n, pc);

	return res;
}

static void set_memory(u12_t n, u4_t v)
{
	if (n < 0x280) {
		/* RAM */
		g_hal->log(LOG_MEMORY, "RAM              - ");
	} else if (n >= 0xE00 && n < 0xE50) {
		/* Display Memory 1 */
		set_lcd(n, v);
		g_hal->log(LOG_MEMORY, "Display Memory 1 - ");
	} else if (n >= 0xE80 && n < 0xED0) {
		/* Display Memory 2 */
		set_lcd(n, v);
		g_hal->log(LOG_MEMORY, "Display Memory 2 - ");
	} else if (n >= 0xF00 && n < 0xF80) {
		/* I/O Memory */
		set_io(n, v);
		g_hal->log(LOG_MEMORY, "I/O              - ");
	} else {
		g_hal->log(LOG_ERROR,   "Write 0x%X to invalid memory address 0x%03X - PC = 0x%04X\n", v, n, pc);
		return;
	}

	/* Cache any data written to a valid address */
	memory[n] = v;

	g_hal->log(LOG_MEMORY, "Write 0x%X - Address 0x%03X - PC = 0x%04X\n", v, n, pc);
}

static u4_t get_rq(u12_t rq)
{
	switch (rq & 0x3) {
		case 0x0:
			return a;

		case 0x1:
			return b;

		case 0x2:
			return M(x);

		case 0x3:
			return M(y);
	}

	return 0;
}

static void set_rq(u12_t rq, u4_t v)
{
	switch (rq & 0x3) {
		case 0x0:
			a = v;
			break;

		case 0x1:
			b = v;
			break;

		case 0x2:
			SET_M(x, v);
			break;

		case 0x3:
			SET_M(y, v);
			break;
	}
}

/* Instructions */
static inline void op_pset_cb(u8_t arg0, u8_t arg1)
{
	np = arg0;
}

static inline void op_jp_cb(u8_t arg0, u8_t arg1)
{
	next_pc = arg0 | (np << 8);
}

static inline void op_jp_c_cb(u8_t arg0, u8_t arg1)
{
	if (flags & FLAG_C) {
		next_pc = arg0 | (np << 8);
	}
}

static inline void op_jp_nc_cb(u8_t arg0, u8_t arg1)
{
	if (!(flags & FLAG_C)) {
		next_pc = arg0 | (np << 8);
	}
}

static inline void op_jp_z_cb(u8_t arg0, u8_t arg1)
{
	if (flags & FLAG_Z) {
		next_pc = arg0 | (np << 8);
	}
}

static inline void op_jp_nz_cb(u8_t arg0, u8_t arg1)
{
	if (!(flags & FLAG_Z)) {
		next_pc = arg0 | (np << 8);
	}
}

static inline void op_jpba_cb(u8_t arg0, u8_t arg1)
{
	next_pc = a | (b << 4) | (np << 8);
}

static inline void op_call_cb(u8_t arg0, u8_t arg1)
{
	pc = (pc + 1) & 0x1FFF; // This does not actually change the PC register
	SET_M(sp - 1, PCP);
	SET_M(sp - 2, PCSH);
	SET_M(sp - 3, PCSL);
	sp = (sp - 3) & 0xFF;
	next_pc = TO_PC(PCB, NPP, arg0);
	call_depth++;
}

static inline void op_calz_cb(u8_t arg0, u8_t arg1)
{
	pc = (pc + 1) & 0x1FFF; // This does not actually change the PC register
	SET_M(sp - 1, PCP);
	SET_M(sp - 2, PCSH);
	SET_M(sp - 3, PCSL);
	sp = (sp - 3) & 0xFF;
	next_pc = TO_PC(PCB, 0, arg0);
	call_depth++;
}

static inline void op_ret_cb(u8_t arg0, u8_t arg1)
{
	next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
	sp = (sp + 3) & 0xFF;
	call_depth--;
}

static inline void op_rets_cb(u8_t arg0, u8_t arg1)
{
	next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
	sp = (sp + 3) & 0xFF;
	next_pc = (pc + 1) & 0x1FFF;
	call_depth--;
}

static inline void op_retd_cb(u8_t arg0, u8_t arg1)
{
	next_pc = M(sp) | (M(sp + 1) << 4) | (M(sp + 2) << 8) | (PCB << 12);
	sp = (sp + 3) & 0xFF;
	SET_M(x, arg0 & 0xF);
	SET_M(x + 1, (arg0 >> 4) & 0xF);
	x = (x + 2) & 0xFFF;
	call_depth--;
}

static inline void op_nop5_cb(u8_t arg0, u8_t arg1)
{
}

static inline void op_nop7_cb(u8_t arg0, u8_t arg1)
{
}

static inline void op_halt_cb(u8_t arg0, u8_t arg1)
{
	g_hal->halt();
}

static inline void op_inc_x_cb(u8_t arg0, u8_t arg1)
{
	x = (x + 1) & 0xFFF;
}

static inline void op_inc_y_cb(u8_t arg0, u8_t arg1)
{
	y = (y + 1) & 0xFFF;
}

static inline void op_ld_x_cb(u8_t arg0, u8_t arg1)
{
	x = arg0 | (XP << 8);
}

static inline void op_ld_y_cb(u8_t arg0, u8_t arg1)
{
	y = arg0 | (YP << 8);
}

static inline void op_ld_xp_r_cb(u8_t arg0, u8_t arg1)
{
	x = XHL | (RQ(arg0) << 8);
}

static inline void op_ld_xh_r_cb(u8_t arg0, u8_t arg1)
{
	x = XL | (RQ(arg0) << 4) | (XP << 8);
}

static inline void op_ld_xl_r_cb(u8_t arg0, u8_t arg1)
{
	x = RQ(arg0) | (XH << 4) | (XP << 8);
}

static inline void op_ld_yp_r_cb(u8_t arg0, u8_t arg1)
{
	y = YHL | (RQ(arg0) << 8);
}

static inline void op_ld_yh_r_cb(u8_t arg0, u8_t arg1)
{
	y = YL | (RQ(arg0) << 4) | (YP << 8);
}

static inline void op_ld_yl_r_cb(u8_t arg0, u8_t arg1)
{
	y = RQ(arg0) | (YH << 4) | (YP << 8);
}

static inline void op_ld_r_xp_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, XP);
}

static inline void op_ld_r_xh_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, XH);
}

static inline void op_ld_r_xl_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, XL);
}

static inline void op_ld_r_yp_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, YP);
}

static inline void op_ld_r_yh_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, YH);
}

static inline void op_ld_r_yl_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, YL);
}

static inline void op_adc_xh_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = XH + arg0 + C;
	x = XL | ((tmp & 0xF) << 4)| (XP << 8);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_adc_xl_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = XL + arg0 + C;
	x = (tmp & 0xF) | (XH << 4) | (XP << 8);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_adc_yh_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = YH + arg0 + C;
	y = YL | ((tmp & 0xF) << 4)| (YP << 8);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_adc_yl_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = YL + arg0 + C;
	y = (tmp & 0xF) | (YH << 4) | (YP << 8);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!(tmp & 0xF)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_xh_cb(u8_t arg0, u8_t arg1)
{
	if (XH < arg0) { SET_C(); } else { CLEAR_C(); }
	if (XH == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_xl_cb(u8_t arg0, u8_t arg1)
{
	if (XL < arg0) { SET_C(); } else { CLEAR_C(); }
	if (XL == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_yh_cb(u8_t arg0, u8_t arg1)
{
	if (YH < arg0) { SET_C(); } else { CLEAR_C(); }
	if (YH == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_yl_cb(u8_t arg0, u8_t arg1)
{
	if (YL < arg0) { SET_C(); } else { CLEAR_C(); }
	if (YL == arg0) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_ld_r_i_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, arg1);
}

static inline void op_ld_r_q_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg1));
}

static inline void op_ld_a_mn_cb(u8_t arg0, u8_t arg1)
{
	a = M(arg0);
}

static inline void op_ld_b_mn_cb(u8_t arg0, u8_t arg1)
{
	b = M(arg0);
}

static inline void op_ld_mn_a_cb(u8_t arg0, u8_t arg1)
{
	SET_M(arg0, a);
}

static inline void op_ld_mn_b_cb(u8_t arg0, u8_t arg1)
{
	SET_M(arg0, b);
}

static inline void op_ldpx_mx_cb(u8_t arg0, u8_t arg1)
{
	SET_M(x, arg0);
	x = (x + 1) & 0xFFF;
}

static inline void op_ldpx_r_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg1));
	x = (x + 1) & 0xFFF;
}

static inline void op_ldpy_my_cb(u8_t arg0, u8_t arg1)
{
	SET_M(y, arg0);
	y = (y + 1) & 0xFFF;
}

static inline void op_ldpy_r_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg1));
	y = (y + 1) & 0xFFF;
}

static inline void op_lbpx_cb(u8_t arg0, u8_t arg1)
{
	SET_M(x, arg0 & 0xF);
	SET_M(x + 1, (arg0 >> 4) & 0xF);
	x = (x + 2) & 0xFFF;
}

static inline void op_set_cb(u8_t arg0, u8_t arg1)
{
	flags |= arg0;
	// This might enable a triggered interrupt to now be asserted.
	interrupt_asserted = 1;
}

static inline void op_rst_cb(u8_t arg0, u8_t arg1)
{
	flags &= arg0;
}

static inline void op_scf_cb(u8_t arg0, u8_t arg1)
{
	SET_C();
}

static inline void op_rcf_cb(u8_t arg0, u8_t arg1)
{
	CLEAR_C();
}

static inline void op_szf_cb(u8_t arg0, u8_t arg1)
{
	SET_Z();
}

static inline void op_rzf_cb(u8_t arg0, u8_t arg1)
{
	CLEAR_Z();
}

static inline void op_sdf_cb(u8_t arg0, u8_t arg1)
{
	SET_D();
}

static inline void op_rdf_cb(u8_t arg0, u8_t arg1)
{
	CLEAR_D();
}

static inline void op_ei_cb(u8_t arg0, u8_t arg1)
{
	SET_I();
	// This might enable a triggered interrupt to now be asserted.
	interrupt_asserted = 1;
}

static inline void op_di_cb(u8_t arg0, u8_t arg1)
{
	CLEAR_I();
	// Even if there's an interrupt triggered, it won't be asserted now.
	interrupt_asserted = 0;
}

static inline void op_inc_sp_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp + 1) & 0xFF;
}

static inline void op_dec_sp_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
}

static inline void op_push_r_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, RQ(arg0));
}

static inline void op_push_xp_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, XP);
}

static inline void op_push_xh_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, XH);
}

static inline void op_push_xl_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, XL);
}

static inline void op_push_yp_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, YP);
}

static inline void op_push_yh_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, YH);
}

static inline void op_push_yl_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, YL);
}

static inline void op_push_f_cb(u8_t arg0, u8_t arg1)
{
	sp = (sp - 1) & 0xFF;
	SET_M(sp, flags);
}

static inline void op_pop_r_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, M(sp));
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_xp_cb(u8_t arg0, u8_t arg1)
{
	x = XL | (XH << 4)| (M(sp) << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_xh_cb(u8_t arg0, u8_t arg1)
{
	x = XL | (M(sp) << 4)| (XP << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_xl_cb(u8_t arg0, u8_t arg1)
{
	x = M(sp) | (XH << 4)| (XP << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_yp_cb(u8_t arg0, u8_t arg1)
{
	y = YL | (YH << 4)| (M(sp) << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_yh_cb(u8_t arg0, u8_t arg1)
{
	y = YL | (M(sp) << 4)| (YP << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_yl_cb(u8_t arg0, u8_t arg1)
{
	y = M(sp) | (YH << 4)| (YP << 8);
	sp = (sp + 1) & 0xFF;
}

static inline void op_pop_f_cb(u8_t arg0, u8_t arg1)
{
	flags = M(sp);
	sp = (sp + 1) & 0xFF;
	// This might enable a triggered interrupt to now be asserted.
	interrupt_asserted = 1;
}

static inline void op_ld_sph_r_cb(u8_t arg0, u8_t arg1)
{
	sp = SPL | (RQ(arg0) << 4);
}

static inline void op_ld_spl_r_cb(u8_t arg0, u8_t arg1)
{
	sp = RQ(arg0) | (SPH << 4);
}

static inline void op_ld_r_sph_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, SPH);
}

static inline void op_ld_r_spl_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, SPL);
}

static inline void op_add_r_i_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) + arg1;
	if (D) {
		if (tmp >= 10) {
			SET_RQ(arg0, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_RQ(arg0, tmp);
			CLEAR_C();
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_add_r_q_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) + RQ(arg1);
	if (D) {
		if (tmp >= 10) {
			SET_RQ(arg0, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_RQ(arg0, tmp);
			CLEAR_C();
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_adc_r_i_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) + arg1 + C;
	if (D) {
		if (tmp >= 10) {
			SET_RQ(arg0, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_RQ(arg0, tmp);
			CLEAR_C();
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_adc_r_q_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) + RQ(arg1) + C;
	if (D) {
		if (tmp >= 10) {
			SET_RQ(arg0, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_RQ(arg0, tmp);
			CLEAR_C();
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_sub_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) - RQ(arg1);
	if (D) {
		if (tmp >> 4) {
			SET_RQ(arg0, (tmp - 6) & 0xF);
		} else {
			SET_RQ(arg0, tmp);
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
	}
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_sbc_r_i_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) - arg1 - C;
	if (D) {
		if (tmp >> 4) {
			SET_RQ(arg0, (tmp - 6) & 0xF);
		} else {
			SET_RQ(arg0, tmp);
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
	}
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_sbc_r_q_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = RQ(arg0) - RQ(arg1) - C;
	if (D) {
		if (tmp >> 4) {
			SET_RQ(arg0, (tmp - 6) & 0xF);
		} else {
			SET_RQ(arg0, tmp);
		}
	} else {
		SET_RQ(arg0, tmp & 0xF);
	}
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_and_r_i_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) & arg1);
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_and_r_q_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) & RQ(arg1));
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_or_r_i_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) | arg1);
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_or_r_q_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) | RQ(arg1));
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_xor_r_i_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) ^ arg1);
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_xor_r_q_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, RQ(arg0) ^ RQ(arg1));
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_r_i_cb(u8_t arg0, u8_t arg1)
{
	if (RQ(arg0) < arg1) { SET_C(); } else { CLEAR_C(); }
	if (RQ(arg0) == arg1) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_cp_r_q_cb(u8_t arg0, u8_t arg1)
{
	if (RQ(arg0) < RQ(arg1)) { SET_C(); } else { CLEAR_C(); }
	if (RQ(arg0) == RQ(arg1)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_fan_r_i_cb(u8_t arg0, u8_t arg1)
{
	if (!(RQ(arg0) & arg1)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_fan_r_q_cb(u8_t arg0, u8_t arg1)
{
	if (!(RQ(arg0) & RQ(arg1))) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_rlc_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = (RQ(arg0) << 1) | C;
	if (RQ(arg0) & 0x8) { SET_C(); } else { CLEAR_C(); }
	SET_RQ(arg0, tmp & 0xF);
	/* No need to set Z (issue in DS) */
}

static inline void op_rrc_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = (RQ(arg0) >> 1) | (C << 3);
	if (RQ(arg0) & 0x1) { SET_C(); } else { CLEAR_C(); }
	SET_RQ(arg0, tmp & 0xF);
	/* No need to set Z (issue in DS) */
}

static inline void op_inc_mn_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(arg0) + 1;
	SET_M(arg0, tmp & 0xF);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!M(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_dec_mn_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(arg0) - 1;
	SET_M(arg0, tmp & 0xF);
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!M(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

static inline void op_acpx_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(x) + RQ(arg0) + C;
	if (D) {
		if (tmp >= 10) {
			SET_M(x, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_M(x, tmp);
			CLEAR_C();
		}
	} else {
		SET_M(x, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!M(x)) { SET_Z(); } else { CLEAR_Z(); }
	x = (x + 1) & 0xFFF;
}

static inline void op_acpy_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(y) + RQ(arg0) + C;
	if (D) {
		if (tmp >= 10) {
			SET_M(y, (tmp - 10) & 0xF);
			SET_C();
		} else {
			SET_M(y, tmp);
			CLEAR_C();
		}
	} else {
		SET_M(y, tmp & 0xF);
		if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	}
	if (!M(y)) { SET_Z(); } else { CLEAR_Z(); }
	y = (y + 1) & 0xFFF;
}

static inline void op_scpx_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(x) - RQ(arg0) - C;
	if (D) {
		if (tmp >> 4) {
			SET_M(x, (tmp - 6) & 0xF);
		} else {
			SET_M(x, tmp);
		}
	} else {
		SET_M(x, tmp & 0xF);
	}
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!M(x)) { SET_Z(); } else { CLEAR_Z(); }
	x = (x + 1) & 0xFFF;
}

static inline void op_scpy_cb(u8_t arg0, u8_t arg1)
{
	u8_t tmp;

	tmp = M(y) - RQ(arg0) - C;
	if (D) {
		if (tmp >> 4) {
			SET_M(y, (tmp - 6) & 0xF);
		} else {
			SET_M(y, tmp);
		}
	} else {
		SET_M(y, tmp & 0xF);
	}
	if (tmp >> 4) { SET_C(); } else { CLEAR_C(); }
	if (!M(y)) { SET_Z(); } else { CLEAR_Z(); }
	y = (y + 1) & 0xFFF;
}

static inline void op_not_cb(u8_t arg0, u8_t arg1)
{
	SET_RQ(arg0, ~RQ(arg0) & 0xF);
	if (!RQ(arg0)) { SET_Z(); } else { CLEAR_Z(); }
}

/* The E0C6S46 supported instructions */
#define OP_TABLE(func) \
func("PSET #0x%02X            "  , 0xE40, MASK_7B , 0, 0    , 5 , pset) /* PSET */ \
func("JP   #0x%02X            "  , 0x000, MASK_4B , 0, 0    , 5 , jp) /* JP */ \
func("JP   C #0x%02X          "  , 0x200, MASK_4B , 0, 0    , 5 , jp_c) /* JP_C */ \
func("JP   NC #0x%02X         "  , 0x300, MASK_4B , 0, 0    , 5 , jp_nc) /* JP_NC */ \
func("JP   Z #0x%02X          "  , 0x600, MASK_4B , 0, 0    , 5 , jp_z) /* JP_Z */ \
func("JP   NZ #0x%02X         "  , 0x700, MASK_4B , 0, 0    , 5 , jp_nz) /* JP_NZ */ \
func("JPBA                  "    , 0xFE8, MASK_12B, 0, 0    , 5 , jpba) /* JPBA */ \
func("CALL #0x%02X            "  , 0x400, MASK_4B , 0, 0    , 7 , call) /* CALL */ \
func("CALZ #0x%02X            "  , 0x500, MASK_4B , 0, 0    , 7 , calz) /* CALZ */ \
func("RET                   "    , 0xFDF, MASK_12B, 0, 0    , 7 , ret) /* RET */ \
func("RETS                  "    , 0xFDE, MASK_12B, 0, 0    , 12, rets) /* RETS */ \
func("RETD #0x%02X            "  , 0x100, MASK_4B , 0, 0    , 12, retd) /* RETD */ \
func("NOP5                  "    , 0xFFB, MASK_12B, 0, 0    , 5 , nop5) /* NOP5 */ \
func("NOP7                  "    , 0xFFF, MASK_12B, 0, 0    , 7 , nop7) /* NOP7 */ \
func("HALT                  "    , 0xFF8, MASK_12B, 0, 0    , 5 , halt) /* HALT */ \
func("INC  X #0x%02X          "  , 0xEE0, MASK_12B, 0, 0    , 5 , inc_x) /* INC_X */ \
func("INC  Y #0x%02X          "  , 0xEF0, MASK_12B, 0, 0    , 5 , inc_y) /* INC_Y */ \
func("LD   X #0x%02X          "  , 0xB00, MASK_4B , 0, 0    , 5 , ld_x) /* LD_X */ \
func("LD   Y #0x%02X          "  , 0x800, MASK_4B , 0, 0    , 5 , ld_y) /* LD_Y */ \
func("LD   XP R(#0x%02X)      "  , 0xE80, MASK_10B, 0, 0    , 5 , ld_xp_r) /* LD_XP_R */ \
func("LD   XH R(#0x%02X)      "  , 0xE84, MASK_10B, 0, 0    , 5 , ld_xh_r) /* LD_XH_R */ \
func("LD   XL R(#0x%02X)      "  , 0xE88, MASK_10B, 0, 0    , 5 , ld_xl_r) /* LD_XL_R */ \
func("LD   YP R(#0x%02X)      "  , 0xE90, MASK_10B, 0, 0    , 5 , ld_yp_r) /* LD_YP_R */ \
func("LD   YH R(#0x%02X)      "  , 0xE94, MASK_10B, 0, 0    , 5 , ld_yh_r) /* LD_YH_R */ \
func("LD   YL R(#0x%02X)      "  , 0xE98, MASK_10B, 0, 0    , 5 , ld_yl_r) /* LD_YL_R */ \
func("LD   R(#0x%02X) XP      "  , 0xEA0, MASK_10B, 0, 0    , 5 , ld_r_xp) /* LD_R_XP */ \
func("LD   R(#0x%02X) XH      "  , 0xEA4, MASK_10B, 0, 0    , 5 , ld_r_xh) /* LD_R_XH */ \
func("LD   R(#0x%02X) XL      "  , 0xEA8, MASK_10B, 0, 0    , 5 , ld_r_xl) /* LD_R_XL */ \
func("LD   R(#0x%02X) YP      "  , 0xEB0, MASK_10B, 0, 0    , 5 , ld_r_yp) /* LD_R_YP */ \
func("LD   R(#0x%02X) YH      "  , 0xEB4, MASK_10B, 0, 0    , 5 , ld_r_yh) /* LD_R_YH */ \
func("LD   R(#0x%02X) YL      "  , 0xEB8, MASK_10B, 0, 0    , 5 , ld_r_yl) /* LD_R_YL */ \
func("ADC  XH #0x%02X         "  , 0xA00, MASK_8B , 0, 0    , 7 , adc_xh) /* ADC_XH */ \
func("ADC  XL #0x%02X         "  , 0xA10, MASK_8B , 0, 0    , 7 , adc_xl) /* ADC_XL */ \
func("ADC  YH #0x%02X         "  , 0xA20, MASK_8B , 0, 0    , 7 , adc_yh) /* ADC_YH */ \
func("ADC  YL #0x%02X         "  , 0xA30, MASK_8B , 0, 0    , 7 , adc_yl) /* ADC_YL */ \
func("CP   XH #0x%02X         "  , 0xA40, MASK_8B , 0, 0    , 7 , cp_xh) /* CP_XH */ \
func("CP   XL #0x%02X         "  , 0xA50, MASK_8B , 0, 0    , 7 , cp_xl) /* CP_XL */ \
func("CP   YH #0x%02X         "  , 0xA60, MASK_8B , 0, 0    , 7 , cp_yh) /* CP_YH */ \
func("CP   YL #0x%02X         "  , 0xA70, MASK_8B , 0, 0    , 7 , cp_yl) /* CP_YL */ \
func("LD   R(#0x%02X) #0x%02X   ", 0xE00, MASK_6B , 4, 0x030, 5 , ld_r_i) /* LD_R_I */ \
func("LD   R(#0x%02X) Q(#0x%02X)", 0xEC0, MASK_8B , 2, 0x00C, 5 , ld_r_q) /* LD_R_Q */ \
func("LD   A M(#0x%02X)       "  , 0xFA0, MASK_8B , 0, 0    , 5 , ld_a_mn) /* LD_A_MN */ \
func("LD   B M(#0x%02X)       "  , 0xFB0, MASK_8B , 0, 0    , 5 , ld_b_mn) /* LD_B_MN */ \
func("LD   M(#0x%02X) A       "  , 0xF80, MASK_8B , 0, 0    , 5 , ld_mn_a) /* LD_MN_A */ \
func("LD   M(#0x%02X) B       "  , 0xF90, MASK_8B , 0, 0    , 5 , ld_mn_b) /* LD_MN_B */ \
func("LDPX MX #0x%02X         "  , 0xE60, MASK_8B , 0, 0    , 5 , ldpx_mx) /* LDPX_MX */ \
func("LDPX R(#0x%02X) Q(#0x%02X)", 0xEE0, MASK_8B , 2, 0x00C, 5 , ldpx_r) /* LDPX_R */ \
func("LDPY MY #0x%02X         "  , 0xE70, MASK_8B , 0, 0    , 5 , ldpy_my) /* LDPY_MY */ \
func("LDPY R(#0x%02X) Q(#0x%02X)", 0xEF0, MASK_8B , 2, 0x00C, 5 , ldpy_r) /* LDPY_R */ \
func("LBPX #0x%02X            "  , 0x900, MASK_4B , 0, 0    , 5 , lbpx) /* LBPX */ \
func("SET  #0x%02X            "  , 0xF40, MASK_8B , 0, 0    , 7 , set) /* SET */ \
func("RST  #0x%02X            "  , 0xF50, MASK_8B , 0, 0    , 7 , rst) /* RST */ \
func("SCF                   "    , 0xF41, MASK_12B, 0, 0    , 7 , scf) /* SCF */ \
func("RCF                   "    , 0xF5E, MASK_12B, 0, 0    , 7 , rcf) /* RCF */ \
func("SZF                   "    , 0xF42, MASK_12B, 0, 0    , 7 , szf) /* SZF */ \
func("RZF                   "    , 0xF5D, MASK_12B, 0, 0    , 7 , rzf) /* RZF */ \
func("SDF                   "    , 0xF44, MASK_12B, 0, 0    , 7 , sdf) /* SDF */ \
func("RDF                   "    , 0xF5B, MASK_12B, 0, 0    , 7 , rdf) /* RDF */ \
func("EI                    "    , 0xF48, MASK_12B, 0, 0    , 7 , ei) /* EI */ \
func("DI                    "    , 0xF57, MASK_12B, 0, 0    , 7 , di) /* DI */ \
func("INC  SP               "    , 0xFDB, MASK_12B, 0, 0    , 5 , inc_sp) /* INC_SP */ \
func("DEC  SP               "    , 0xFCB, MASK_12B, 0, 0    , 5 , dec_sp) /* DEC_SP */ \
func("PUSH R(#0x%02X)         "  , 0xFC0, MASK_10B, 0, 0    , 5 , push_r) /* PUSH_R */ \
func("PUSH XP               "    , 0xFC4, MASK_12B, 0, 0    , 5 , push_xp) /* PUSH_XP */ \
func("PUSH XH               "    , 0xFC5, MASK_12B, 0, 0    , 5 , push_xh) /* PUSH_XH */ \
func("PUSH XL               "    , 0xFC6, MASK_12B, 0, 0    , 5 , push_xl) /* PUSH_XL */ \
func("PUSH YP               "    , 0xFC7, MASK_12B, 0, 0    , 5 , push_yp) /* PUSH_YP */ \
func("PUSH YH               "    , 0xFC8, MASK_12B, 0, 0    , 5 , push_yh) /* PUSH_YH */ \
func("PUSH YL               "    , 0xFC9, MASK_12B, 0, 0    , 5 , push_yl) /* PUSH_YL */ \
func("PUSH F                "    , 0xFCA, MASK_12B, 0, 0    , 5 , push_f) /* PUSH_F */ \
func("POP  R(#0x%02X)         "  , 0xFD0, MASK_10B, 0, 0    , 5 , pop_r) /* POP_R */ \
func("POP  XP               "    , 0xFD4, MASK_12B, 0, 0    , 5 , pop_xp) /* POP_XP */ \
func("POP  XH               "    , 0xFD5, MASK_12B, 0, 0    , 5 , pop_xh) /* POP_XH */ \
func("POP  XL               "    , 0xFD6, MASK_12B, 0, 0    , 5 , pop_xl) /* POP_XL */ \
func("POP  YP               "    , 0xFD7, MASK_12B, 0, 0    , 5 , pop_yp) /* POP_YP */ \
func("POP  YH               "    , 0xFD8, MASK_12B, 0, 0    , 5 , pop_yh) /* POP_YH */ \
func("POP  YL               "    , 0xFD9, MASK_12B, 0, 0    , 5 , pop_yl) /* POP_YL */ \
func("POP  F                "    , 0xFDA, MASK_12B, 0, 0    , 5 , pop_f) /* POP_F */ \
func("LD   SPH R(#0x%02X)     "  , 0xFE0, MASK_10B, 0, 0    , 5 , ld_sph_r) /* LD_SPH_R */ \
func("LD   SPL R(#0x%02X)     "  , 0xFF0, MASK_10B, 0, 0    , 5 , ld_spl_r) /* LD_SPL_R */ \
func("LD   R(#0x%02X) SPH     "  , 0xFE4, MASK_10B, 0, 0    , 5 , ld_r_sph) /* LD_R_SPH */ \
func("LD   R(#0x%02X) SPL     "  , 0xFF4, MASK_10B, 0, 0    , 5 , ld_r_spl) /* LD_R_SPL */ \
func("ADD  R(#0x%02X) #0x%02X   ", 0xC00, MASK_6B , 4, 0x030, 7 , add_r_i) /* ADD_R_I */ \
func("ADD  R(#0x%02X) Q(#0x%02X)", 0xA80, MASK_8B , 2, 0x00C, 7 , add_r_q) /* ADD_R_Q */ \
func("ADC  R(#0x%02X) #0x%02X   ", 0xC40, MASK_6B , 4, 0x030, 7 , adc_r_i) /* ADC_R_I */ \
func("ADC  R(#0x%02X) Q(#0x%02X)", 0xA90, MASK_8B , 2, 0x00C, 7 , adc_r_q) /* ADC_R_Q */ \
func("SUB  R(#0x%02X) Q(#0x%02X)", 0xAA0, MASK_8B , 2, 0x00C, 7 , sub) /* SUB */ \
func("SBC  R(#0x%02X) #0x%02X   ", 0xB40, MASK_6B , 4, 0x030, 7 , sbc_r_i) /* SBC_R_I */ \
func("SBC  R(#0x%02X) Q(#0x%02X)", 0xAB0, MASK_8B , 2, 0x00C, 7 , sbc_r_q) /* SBC_R_Q */ \
func("AND  R(#0x%02X) #0x%02X   ", 0xC80, MASK_6B , 4, 0x030, 7 , and_r_i) /* AND_R_I */ \
func("AND  R(#0x%02X) Q(#0x%02X)", 0xAC0, MASK_8B , 2, 0x00C, 7 , and_r_q) /* AND_R_Q */ \
func("OR   R(#0x%02X) #0x%02X   ", 0xCC0, MASK_6B , 4, 0x030, 7 , or_r_i) /* OR_R_I */ \
func("OR   R(#0x%02X) Q(#0x%02X)", 0xAD0, MASK_8B , 2, 0x00C, 7 , or_r_q) /* OR_R_Q */ \
func("XOR  R(#0x%02X) #0x%02X   ", 0xD00, MASK_6B , 4, 0x030, 7 , xor_r_i) /* XOR_R_I */ \
func("XOR  R(#0x%02X) Q(#0x%02X)", 0xAE0, MASK_8B , 2, 0x00C, 7 , xor_r_q) /* XOR_R_Q */ \
func("CP   R(#0x%02X) #0x%02X   ", 0xDC0, MASK_6B , 4, 0x030, 7 , cp_r_i) /* CP_R_I */ \
func("CP   R(#0x%02X) Q(#0x%02X)", 0xF00, MASK_8B , 2, 0x00C, 7 , cp_r_q) /* CP_R_Q */ \
func("FAN  R(#0x%02X) #0x%02X   ", 0xD80, MASK_6B , 4, 0x030, 7 , fan_r_i) /* FAN_R_I */ \
func("FAN  R(#0x%02X) Q(#0x%02X)", 0xF10, MASK_8B , 2, 0x00C, 7 , fan_r_q) /* FAN_R_Q */ \
func("RLC  R(#0x%02X)         "  , 0xAF0, MASK_8B , 0, 0    , 7 , rlc) /* RLC */ \
func("RRC  R(#0x%02X)         "  , 0xE8C, MASK_10B, 0, 0    , 5 , rrc) /* RRC */ \
func("INC  M(#0x%02X)         "  , 0xF60, MASK_8B , 0, 0    , 7 , inc_mn) /* INC_MN */ \
func("DEC  M(#0x%02X)         "  , 0xF70, MASK_8B , 0, 0    , 7 , dec_mn) /* DEC_MN */ \
func("ACPX R(#0x%02X)         "  , 0xF28, MASK_10B, 0, 0    , 7 , acpx) /* ACPX */ \
func("ACPY R(#0x%02X)         "  , 0xF2C, MASK_10B, 0, 0    , 7 , acpy) /* ACPY */ \
func("SCPX R(#0x%02X)         "  , 0xF38, MASK_10B, 0, 0    , 7 , scpx) /* SCPX */ \
func("SCPY R(#0x%02X)         "  , 0xF3C, MASK_10B, 0, 0    , 7 , scpy) /* SCPY */ \
func("NOT  R(#0x%02X)         "  , 0xD0F, 0xFCF   , 4, 0    , 7 , not) /* NOT */

static const op_t ops[] = {
#define STRUCT_ENTRY(log, code, mask, shift_arg0, mask_arg0, cycles, name) \
	{ log, code, mask, shift_arg0, mask_arg0, cycles, &op_##name##_cb },
OP_TABLE(STRUCT_ENTRY)
	{NULL, 0, 0, 0, 0, 0, NULL},
};

static timestamp_t wait_for_cycles(timestamp_t since, u8_t cycles) {
	timestamp_t deadline;

	tick_counter += cycles;

	if (speed_ratio == 0) {
		/* Emulation will be as fast as possible */
		return g_hal->get_timestamp();
	}

	deadline = since + (cycles * ts_freq)/(TICK_FREQUENCY * speed_ratio);
	g_hal->sleep_until(deadline);

	return deadline;
}

static u32_t check_timer_interrupts(void) {
	/* Handle timers using the internal tick counter */
	if (tick_counter - clk_timer_timestamp >= TIMER_1HZ_PERIOD) {
		do {
			clk_timer_timestamp += TIMER_1HZ_PERIOD;
		} while (tick_counter - clk_timer_timestamp >= TIMER_1HZ_PERIOD);

		generate_interrupt(INT_CLOCK_TIMER_SLOT, 3);
	}

	if (prog_timer_enabled && tick_counter - prog_timer_timestamp >= TIMER_256HZ_PERIOD) {
		do {
			prog_timer_timestamp += TIMER_256HZ_PERIOD;
			prog_timer_data--;

			if (prog_timer_data == 0) {
				prog_timer_data = prog_timer_rld;
				generate_interrupt(INT_PROG_TIMER_SLOT, 0);
			}
		} while (tick_counter - prog_timer_timestamp >= TIMER_256HZ_PERIOD);
	}
	// return the time of the earliest possible next timer interrupt.
	u32_t next_clk = clk_timer_timestamp + TIMER_1HZ_PERIOD;
	// if the prog_timer isn't currently enabled, then it can't possibly
	// happen sooner than TIMER_256HZ_PERIOD from now, since
	// prog_timer_timestamp will get reset when the prog_timer is enabled
	u32_t next_prog =
		(prog_timer_enabled ? prog_timer_timestamp : tick_counter) +
		TIMER_256HZ_PERIOD;
	return next_clk < next_prog ? next_clk : next_prog;
}

static int process_interrupts(void)
{
	u8_t i;
	// When we return, interrupt_asserted will be false: either there
	// wasn't really an interrupt, or else we'll have cleared I
	interrupt_asserted = 0;

	if (!I) {
		return 0; // Interrupts aren't enabled
	}

	/* Process interrupts in priority order */
	for (i = 0; i < INT_SLOT_NUM; i++) {
		if (interrupts[i].triggered) {
			//printf("IT %u !\n", i);
			SET_M(sp - 1, PCP);
			SET_M(sp - 2, PCSH);
			SET_M(sp - 3, PCSL);
			sp = (sp - 3) & 0xFF;
			CLEAR_I();
			np = TO_NP(NBP, 1);
			pc = TO_PC(PCB, 1, interrupts[i].vector);
			call_depth++;

			ref_ts = wait_for_cycles(ref_ts, 12);
			interrupts[i].triggered = 0;
			return 1;
		}
	}
	return 0; // no interrupts after all.
}

static void print_state(u8_t op_num, u12_t op, u13_t addr)
{
	u8_t i;

	if (!g_hal->is_log_enabled(LOG_CPU)) {
		return;
	}
	if (op_num == 0) {
		for (i = 0; ops[i].log != NULL; i++) {
			if ((op & ops[i].mask) == ops[i].code) {
				op_num = i;
				break;
			}
		}
	}

	g_hal->log(LOG_CPU, "0x%04X: ", addr);

	for (i = 0; i < call_depth; i++) {
		g_hal->log(LOG_CPU, "  ");
	}

	if (ops[op_num].mask_arg0 != 0) {
		/* Two arguments */
		g_hal->log(LOG_CPU, ops[op_num].log, (op & ops[op_num].mask_arg0) >> ops[op_num].shift_arg0, op & ~(ops[op_num].mask | ops[op_num].mask_arg0));
	} else {
		/* One argument */
		g_hal->log(LOG_CPU, ops[op_num].log, (op & ~ops[op_num].mask) >> ops[op_num].shift_arg0);
	}

	if (call_depth < 10) {
		for (i = 0; i < (10 - call_depth); i++) {
			g_hal->log(LOG_CPU, "  ");
		}
	}

	g_hal->log(LOG_CPU, " ; 0x%03X - ", op);
	for (i = 0; i < 12; i++) {
		g_hal->log(LOG_CPU, "%s", ((op >> (11 - i)) & 0x1) ? "1" : "0");
	}
	g_hal->log(LOG_CPU, " - PC = 0x%04X, SP = 0x%02X, NP = 0x%02X, X = 0x%03X, Y = 0x%03X, A = 0x%X, B = 0x%X, F = 0x%X\n", pc, sp, np, x, y, a, b, flags);
}

bool_t cpu_init(const u12_t *program, breakpoint_t *breakpoints, u32_t freq)
{
	u13_t i;

	/* Registers and variables init */
	pc = TO_PC(0, 1, 0x00); // PC starts at bank 0, page 1, step 0
	np = TO_NP(0, 1); // NP starts at page 1
	a = 0; // undef
	b = 0; // undef
	x = 0; // undef
	y = 0; // undef
	sp = 0; // undef
	flags = 0;

	/* Init RAM to zeros */
	for (i = 0; i < MEMORY_SIZE; i++) {
		memory[i] = 0;
	}

	memory[0xF54] = 0xF; // Output port (R40-R43)
	memory[0xF71] = 0x8; // LCD control
	/* TODO: Input relation register */

	g_program = program;
	g_breakpoints = breakpoints;

	ts_freq = freq;
	cpu_sync_ref_timestamp();

	return 0;
}

void cpu_release(void)
{
}

void *threaded_program[PROGRAM_SIZE] = { 0 };
uint8_t threaded_arg0[PROGRAM_SIZE];
uint8_t threaded_arg1[PROGRAM_SIZE];
uint8_t threaded_cycles[PROGRAM_SIZE];

int cpu_fast_step() {
	static void *threaded_ops[] = {
#define LABEL_ENTRY(log, code, mask, shift_arg0, mask_arg0, cycles, name) \
		&&l_##name,
OP_TABLE(LABEL_ENTRY)
	};
	uint8_t arg0 = 0, arg1 = 0;
	// Precompute and cache some stuff that's not going to change
	// before this step is over.
	bool_t skip_first_breakpoint = 0;
	bool_t log_enabled = g_hal->is_log_enabled(LOG_CPU);
	bool_t fast_as_possible = (speed_ratio == 0);
	timestamp_t adj_cycles_5 = 0, adj_cycles_7 = 0, adj_cycles_12 = 0;
	if (!fast_as_possible) {
		adj_cycles_5 = (5 * ts_freq) / (TICK_FREQUENCY * speed_ratio);
		adj_cycles_7 = (7 * ts_freq) / (TICK_FREQUENCY * speed_ratio);
		adj_cycles_12 = (12 * ts_freq) / (TICK_FREQUENCY * speed_ratio);
	}
	// We might have saved/loaded state before calling this, so interrupt
	// asserted might not be valid.  Set it on the first step to
	// re-validate.
	interrupt_asserted = 1;
	// Ditto for soonest_interrupt; ensure we re-validate it on first step.
	u32_t soonest_interrupt = tick_counter;

	if (threaded_program[0] == NULL) {
		/* init */
		for (int i=0; i<PROGRAM_SIZE; i++) {
			threaded_program[i] = &&l_decode;
		}
	} else {
		// Use l_decode to mark breakpoints; this has the advantage that
		// they will transparently be replaced with the proper decoded
		// label if breakpoints change the next time this is called.
		for (breakpoint_t *bp = g_breakpoints; bp!=NULL; bp = bp->next) {
			threaded_program[bp->addr] = &&l_decode;
			if (pc == bp->addr) {
				skip_first_breakpoint = 1;
			}
		}
	}
	goto *(threaded_program[pc]);

	l_decode:
	{
		u12_t op = g_program[pc];
		void *decoded_op;
		int i;
		// Is this a breakpoint? (but we can step through first breakpoint)
		if (!skip_first_breakpoint) {
			for (breakpoint_t *bp = g_breakpoints; bp!=NULL; bp = bp->next) {
				if (pc==bp->addr) {
					goto l_bailout;
				}
			}
		}
		/* Lookup the OP code */
		for (i = 0; ops[i].log != NULL; i++) {
			if ((op & ops[i].mask) == ops[i].code) {
				break;
			}
		}
		if (ops[i].log == NULL) {
			goto l_bailout; // Illegal operation
		}
		threaded_program[pc] = decoded_op = threaded_ops[i];
		if (ops[i].cb != NULL) {
			if (ops[i].mask_arg0 != 0) {
				/* Two arguments */
				threaded_arg0[pc] = (op & ops[i].mask_arg0) >> ops[i].shift_arg0;
				threaded_arg1[pc] = op & ~(ops[i].mask | ops[i].mask_arg0);
			} else {
				threaded_arg0[pc] = (op & ~ops[i].mask) >> ops[i].shift_arg0;
				threaded_arg1[pc] = 0;
			}
		}
		if (skip_first_breakpoint) {
			skip_first_breakpoint = 0;
			threaded_program[pc] = &&l_decode;
		}
		// now dispatch to that decoded op.
		goto *decoded_op;
	}

#define OP_IMPL(log, code, mask, shift_arg0, mask_arg0, cycles, name) \
	l_##name:														  \
		if (log_enabled) {											  \
			print_state(0, g_program[pc], pc);						  \
		}															  \
		next_pc = (pc + 1) & 0x1FFF;								  \
		arg0 = threaded_arg0[pc];									  \
		if (mask_arg0 != 0) {										  \
			arg1 = threaded_arg1[pc];								  \
		}															  \
		/* This is an unrolled version of wait_for_cycles */          \
		if (!fast_as_possible) {									  \
			timestamp_t deadline = ref_ts + adj_cycles_##cycles;	  \
			g_hal->sleep_until(deadline);						      \
			ref_ts = deadline;										  \
		}															  \
		/* We sleep_until just *before* execution so that we're in */ \
		/* a consistent architectural state in case we save/load */	  \
		/* state from within sleep_until(). */						  \
		tick_counter += cycles;										  \
		op_##name##_cb(arg0, arg1);									  \
		pc = next_pc;												  \
		if (code != 0xE40) {										  \
			/* OP code is not PSET, reset NP */						  \
			np = (pc >> 8) & 0x1F;									  \
		}															  \
		if (tick_counter >= soonest_interrupt) {					  \
			soonest_interrupt = check_timer_interrupts();			  \
		}															  \
		if (interrupt_asserted && code != 0xE40) {					  \
			if (process_interrupts()) {								  \
				goto l_finish_step;									  \
			}														  \
		}															  \
		goto *(threaded_program[pc]);
OP_TABLE(OP_IMPL)

	l_finish_step:
	if (fast_as_possible) {
		ref_ts = g_hal->get_timestamp();
	}
	return 0;

	l_bailout:
	if (fast_as_possible) {
		ref_ts = g_hal->get_timestamp();
	}
	return 1;
}

int cpu_step(void)
{
	u12_t op;
	u8_t i;
	breakpoint_t *bp = g_breakpoints;
	static u8_t previous_cycles = 0;

	op = g_program[pc];

	/* Lookup the OP code */
	for (i = 0; ops[i].log != NULL; i++) {
		if ((op & ops[i].mask) == ops[i].code) {
			break;
		}
	}

	if (ops[i].log == NULL) {
		g_hal->log(LOG_ERROR, "Unknown op-code 0x%X (pc = 0x%04X)\n", op, pc);
		return 1;
	}

	next_pc = (pc + 1) & 0x1FFF;

	/* Display operation along with the current state of the processor */
	print_state(i, op, pc);

	/* Match the speed of the real processor
	 * NOTE: For better accuracy, the final wait should happen here, however
	 * the downside is that all interrupts will likely be delayed by one OP
	 */
	ref_ts = wait_for_cycles(ref_ts, previous_cycles);

	/* Process the OP code */
	if (ops[i].cb != NULL) {
		if (ops[i].mask_arg0 != 0) {
			/* Two arguments */
			ops[i].cb((op & ops[i].mask_arg0) >> ops[i].shift_arg0, op & ~(ops[i].mask | ops[i].mask_arg0));
		} else {
			/* One arguments */
			ops[i].cb((op & ~ops[i].mask) >> ops[i].shift_arg0, 0);
		}
	}

	/* Prepare for the next instruction */
	pc = next_pc;
	previous_cycles = ops[i].cycles;

	if (i > 0) {
		/* OP code is not PSET, reset NP */
		np = (pc >> 8) & 0x1F;
	}

	/* Handle timers using the internal tick counter */
	check_timer_interrupts();

	/* Check if there is any pending interrupt */
	if (i > 0) { // Do not process interrupts after a PSET operation
		process_interrupts();
	}

	/* Check if we could pause the execution */
	while (bp != NULL) {
		if (bp->addr == pc) {
			return 1;
		}

		bp = bp->next;
	}

	return 0;
}
