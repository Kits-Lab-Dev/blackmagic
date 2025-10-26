/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022-2023 1BitSquared <info@1bitsquared.com>
 * Modified by Rachel Mant <git@dragonmux.network>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the low-level JTAG TAP interface.  */

#include <stdio.h>

#include "general.h"
#include "platform.h"
#include "jtagtap.h"
#include "adiv5.h"
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/dwt.h>

jtag_proc_s jtag_proc;

// Вспомогательная функция для задержки в циклах CPU
static inline void delay_cycles(uint32_t cycles)
{
	const uint32_t start = DWT_CYCCNT;
	while ((DWT_CYCCNT - start) < cycles)
		__asm__ volatile("nop");
}

// Упрощённая функция одного бита JTAG
static bool jtagtap_next(bool tms, bool tdi)
{
	gpio_set_val(TMS_PORT, TMS_PIN, tms);
	gpio_set_val(TDI_PORT, TDI_PIN, tdi);
	delay_cycles(1);
	
	// TCK rising edge
	gpio_set(TCK_PORT, TCK_PIN);
	delay_cycles(target_clk_divider);

	// Sample TDO on rising edge
	const bool tdo = gpio_get(TDO_PORT, TDO_PIN) != 0;

	// TCK falling edge
	gpio_clear(TCK_PORT, TCK_PIN);
	delay_cycles(1);

	return tdo;
}

// TMS sequence — отправка до 32 бит
static void jtagtap_tms_seq(uint32_t tms_states, size_t clock_cycles)
{
	gpio_set(TDI_PORT, TDI_PIN); // TDI = 1 during TMS sequences (per spec)
	for (size_t i = 0; i < clock_cycles; ++i) {
		const bool tms = (tms_states >> i) & 1U;
		(void)jtagtap_next(tms, true);
	}
}

// TDI → TDO transfer
static void jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t clock_cycles)
{
	uint8_t value = 0;
	for (size_t i = 0; i < clock_cycles; ++i) {
		const uint8_t bit_idx = i & 7U;
		const size_t byte_idx = i >> 3U;

		const bool tms = (i + 1U == clock_cycles) && final_tms;
		const bool tdi = (data_in[byte_idx] >> bit_idx) & 1U;
		const bool tdo = jtagtap_next(tms, tdi);
		value |= ((tdo & 1U) << bit_idx);

		if (bit_idx == 7U || i + 1U == clock_cycles) {
			data_out[byte_idx] = value;
			value = 0;
		}
	}
}

// TDI only (no TDO read)
static void jtagtap_tdi_seq(bool final_tms, const uint8_t *data_in, size_t clock_cycles)
{
	for (size_t i = 0; i < clock_cycles; ++i) {
		const uint8_t bit_idx = i & 7U;
		const size_t byte_idx = i >> 3U;

		const bool tms = (i + 1U == clock_cycles) && final_tms;
		const bool tdi = (data_in[byte_idx] >> bit_idx) & 1U;
		(void)jtagtap_next(tms, tdi);
	}
}

// Повтор одного состояния (TMS, TDI) N раз
static void jtagtap_cycle(bool tms, bool tdi, size_t clock_cycles)
{
	for (size_t i = 0; i < clock_cycles; ++i) {
		(void)jtagtap_next(tms, tdi);
	}
}

// Сброс через TRST или программный
static void jtagtap_reset(void)
{
#ifdef TRST_PORT
	if (platform_hwversion() == 0) {
		gpio_clear(TRST_PORT, TRST_PIN);
		delay_cycles(80000); // ~1ms @ 80MHz
		gpio_set(TRST_PORT, TRST_PIN);
		delay_cycles(80000);
	}
#endif
	jtagtap_soft_reset();
}

// Инициализация
void jtagtap_init(void)
{
	platform_target_clk_output_enable(true);
	TMS_SET_MODE();

	jtag_proc.jtagtap_reset = jtagtap_reset;
	jtag_proc.jtagtap_next = jtagtap_next;
	jtag_proc.jtagtap_tms_seq = jtagtap_tms_seq;
	jtag_proc.jtagtap_tdi_tdo_seq = jtagtap_tdi_tdo_seq;
	jtag_proc.jtagtap_tdi_seq = jtagtap_tdi_seq;
	jtag_proc.jtagtap_cycle = jtagtap_cycle;
	jtag_proc.tap_idle_cycles = 1;

	// Перевод из SWD в JTAG (как в оригинале)
	jtagtap_cycle(true, false, 51U);
	jtagtap_tms_seq(ADIV5_SWD_TO_JTAG_SELECT_SEQUENCE, 16U);
	jtagtap_cycle(true, false, 51U);
	jtagtap_tms_seq(ADIV5_SWD_TO_DORMANT_SEQUENCE, 16U);
	jtagtap_tms_seq(0xffU, 8U);
	jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_0, 32U);
	jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_1, 32U);
	jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_2, 32U);
	jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_3, 32U);
	jtagtap_tms_seq(ADIV5_ACTIVATION_CODE_ARM_JTAG_DP << 4U, 12U);
}