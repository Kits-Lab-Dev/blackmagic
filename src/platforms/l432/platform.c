/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022-2024 1BitSquared <info@1bitsquared.com>
 * Portions (C) 2020-2021 Stoyan Shopov <stoyan.shopov@gmail.com>
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

/*
 * This file provides the platform specific functions for the STM32L432KCU6 implementation.
 */

#include "general.h"
#include "platform.h"
#include "usb.h"
#include "aux_serial.h"
#include "gdb_if.h"

#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/cortex.h>

static uint32_t hw_version = 100;

int platform_hwversion(void)
{
	return hw_version;
}

void platform_nrst_set_val(bool assert)
{
	gpio_set_val(NRST_PORT, NRST_PIN, assert);
	if (assert)
	{
		for (volatile size_t i = 0; i < 10000; i++)
			continue;
	}
}

bool platform_nrst_get_val()
{
	return gpio_get(NRST_PORT, NRST_PIN) != 0;
}

/* One regular conversion. EOS stays set after a conversion, clear it or the wait returns at once with stale data */
static uint32_t adc_sample(uint8_t channel)
{
	adc_set_regular_sequence(TARGET_V_ADC, 1, &channel);
	ADC_ISR(TARGET_V_ADC) = ADC_ISR_EOC | ADC_ISR_EOS;
	adc_start_conversion_regular(TARGET_V_ADC);
	while (!adc_eos(TARGET_V_ADC))
		continue;
	return adc_read_regular(TARGET_V_ADC);
}

/* Our own 3.3V rail (VDDA = VREF+), from VREFINT calibrated at VDDA = 3.0V */
static uint32_t rail_mv(void)
{
	const uint32_t raw = adc_sample(ADC_CHANNEL_VREF);
	return raw ? (3000U * ST_VREFINT_CAL) / raw : 0U;
}

/* T_VDD behind the R9/R10 divider by 2, the ADC reference being the rail */
static uint32_t target_mv(const uint32_t rail)
{
	return (adc_sample(TARGET_V_CH) * rail * 2U) / 4095U;
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	const uint32_t tenths = (target_mv(rail_mv()) + 50U) / 100U;
	ret[0] = (char)('0' + MIN(tenths / 10U, 9U));
	ret[2] = (char)('0' + tenths % 10U);
	return ret;
}

uint32_t platform_target_voltage_sense(void)
{
	return target_mv(rail_mv()) / 100U;
}

#define BOOTMAGIC0 UINT32_C(0xb007da7a)
#define BOOTMAGIC1 UINT32_C(0xbaadfeed)

/* Survives the system reset: .noinit is not touched by the startup code */
static volatile uint32_t magic[2] __attribute__((section(".noinit")));

/*
 * Write the bootloader flag and reboot, platform_init() then enters the ST system
 * bootloader (USB DFU). There is no BMD bootloader on this board.
 */
void platform_request_boot(void)
{
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
	scb_reset_system();
}

#define ADC_CR_BITS_PROPERTY_RS (ADC_CR_ADCAL | ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_JADSTART | ADC_CR_JADSTP | ADC_CR_ADSTART | ADC_CR_ADSTP)

void platform_init(void)
{
	rcc_periph_clock_enable(RCC_SYSCFG);
	if (magic[0] == BOOTMAGIC0 && magic[1] == BOOTMAGIC1) {
		magic[0] = 0;
		magic[1] = 0;
		/*
		 * Jump to the built in bootloader by mapping System flash at 0 and resetting the core only.
		 * As we just came out of reset, no other deinit is needed.
		 */
		SYSCFG_MEMRMP = (SYSCFG_MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE_MASK) | SYSCFG_MEMRMP_MEM_MODE_SYSTEM;
		scb_reset_core();
	}
	/* The bootloader may start us with System flash still mapped at 0 */
	if ((SYSCFG_MEMRMP & SYSCFG_MEMRMP_MEM_MODE_MASK) == SYSCFG_MEMRMP_MEM_MODE_SYSTEM)
		SYSCFG_MEMRMP &= ~SYSCFG_MEMRMP_MEM_MODE_MASK;

	SCB_VTOR = (uintptr_t)&vector_table;

	rcc_clock_setup_pll(&rcc_hsi16_configs[RCC_CLOCK_VRANGE1_80MHZ]);

	flash_dcache_enable();
	flash_icache_enable();
	flash_prefetch_enable();
	flash_set_ws(FLASH_ACR_LATENCY_4WS);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_SYSCFG);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOH);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_ADC);

	dwt_enable_cycle_counter();

	gpio_mode_setup(LED_PORT_ERROR, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR);
	gpio_set_output_options(LED_PORT_ERROR, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, LED_ERROR);

	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);
	gpio_set_output_options(LED_PORT_UART, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, LED_UART);

	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_IDLE_RUN);
	gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, LED_IDLE_RUN);

	/* Initialize ADC. */
	gpio_mode_setup(TARGET_V_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, TARGET_V_PIN);
	RCC_CCIPR &= ~(RCC_CCIPR_ADCSEL_MASK << RCC_CCIPR_ADCSEL_SHIFT);
	RCC_CCIPR |= (RCC_CCIPR_ADCSEL_SYSCLK << RCC_CCIPR_ADCSEL_SHIFT);

	// Выход из deep power down mode
	ADC_CR(TARGET_V_ADC) &= ~(ADC_CR_DEEPPWD | ADC_CR_BITS_PROPERTY_RS);
	// Включение внутреннего регулятора напряжения
	ADC_CR(TARGET_V_ADC) |= ADC_CR_ADVREGEN;
	// Задержка для стабилизации регулятора
	for (int i = 0; i < 10000; i++)
		__asm__("nop");

	adc_calibrate(TARGET_V_ADC);

	adc_set_resolution(TARGET_V_ADC, ADC_CFGR1_RES_12_BIT);
	adc_set_right_aligned(TARGET_V_ADC);
	adc_set_sample_time_on_all_channels(TARGET_V_ADC, ADC_SMPR_SMP_247DOT5CYC);
	/* VREFINT needs at least 4us of sampling */
	adc_set_sample_time(TARGET_V_ADC, ADC_CHANNEL_VREF, ADC_SMPR_SMP_640DOT5CYC);
	adc_enable_vrefint();

	adc_power_on(TARGET_V_ADC);
	for (int i = 0; i < 100000; i++)
		__asm__("nop");

	gpio_mode_setup(BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, BTN_PIN);
	exti_select_source(BTN_PIN, BTN_PORT);
	exti_set_trigger(BTN_PIN, EXTI_TRIGGER_RISING);
	nvic_enable_irq(BTN_IRQ);
	exti_enable_request(BTN_PIN);

	gpio_set_output_options(NRST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, NRST_PIN);
	gpio_mode_setup(NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRST_PIN);
	gpio_clear(NRST_PORT, NRST_PIN);

	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN);
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TMS_PIN);

	gpio_mode_setup(TDI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDI_PIN);
	gpio_set_output_options(TDI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TDI_PIN);

	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TDO_PIN);

	gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
	gpio_set_output_options(TCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TCK_PIN);
	
	gpio_mode_setup(TMS_DIR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_DIR_PIN);
	gpio_set_output_options(TMS_DIR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TMS_DIR_PIN);
	gpio_set(TMS_DIR_PORT, TMS_DIR_PIN);

	gpio_set(PWR_EN_PORT, PWR_EN_PIN);
	gpio_mode_setup(PWR_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_EN_PIN);
	gpio_set_output_options(PWR_EN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_VERYHIGH, PWR_EN_PIN);

	platform_timing_init();

	blackmagic_usb_init();

	aux_serial_init();

	/* By default, do not drive the swd bus too fast. */
	platform_max_frequency_set(2000000);
}

void platform_target_clk_output_enable(bool enable)
{
	/* Regardless of swdptap.c, tri-state TCK and TMS */
	if (enable)
	{
		gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
		gpio_set_output_options(TCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TCK_PIN);
		SWDIO_MODE_DRIVE();
	}
	else
	{
		gpio_mode_setup(TCK_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TCK_PIN);
		SWDIO_MODE_FLOAT();
	}
}

void platform_ospeed_update(const uint32_t frequency)
{
	// if (frequency > 2000000U)
	// 	PIN_MODE_FAST();
	// else
	// 	PIN_MODE_NORMAL();
}

#ifdef PLATFORM_HAS_POWER_SWITCH
/*
 * Soft start of the target supply. Q1/Q2 connect T_VDD straight to our 3.3V rail (XC6219, 22uF), and a target
 * like Vagonka carries ~145uF: switching on at once shares the charge, the rail collapses and we reset.
 *
 * Instead PWR_EN is pulsed. A short low pulse only partly charges the gate (in through R4 1k2, back through
 * R3 10k), so each pulse lets a bounded charge through the FETs. Our rail is measured right after every pulse
 * (VREFINT): the pulse grows while the droop stays under TPWR_DROOP_MV and shrinks when it goes over. Once
 * T_VDD reaches TPWR_DONE_MV the switch is left on. Derivation of the numbers: firmware/README.md of the board.
 */
#define TPWR_PERIOD_US     50U   /* gate recovers through 10k (~10us) and the LDO refills the 22uF */
#define TPWR_PULSE_MIN_CYC 8U    /* 100ns at 80MHz, too short to open the FETs */
#define TPWR_PULSE_MAX_US  25U   /* by then the FETs are fully on anyway */
#define TPWR_DROOP_MV      200U  /* rail droop allowed per pulse, ~3.3uC from 22uF */
#define TPWR_DONE_MV       3000U /* T_VDD at which the switch stays on */
#define TPWR_TIMEOUT_MS    500U  /* give up and switch off: shorted or too heavy target */
#define TPWR_NRST_HOLD_MS  5U    /* target nRST stays asserted this long after the supply is up */

static void pwr_en_pulse(const uint32_t cycles)
{
	/* Masked: an interrupt inside the pulse would hold the FETs fully on */
	const uint32_t primask = cm_mask_interrupts(1);
	const uint32_t start = DWT_CYCCNT;
	GPIO_BRR(PWR_EN_PORT) = PWR_EN_PIN;
	while (DWT_CYCCNT - start < cycles)
		continue;
	GPIO_BSRR(PWR_EN_PORT) = PWR_EN_PIN;
	cm_mask_interrupts(primask);
}

static bool target_power_soft_start(void)
{
	const uint32_t cycles_per_us = rcc_ahb_frequency / 1000000U;
	const uint32_t period = TPWR_PERIOD_US * cycles_per_us;
	const uint32_t pulse_max = TPWR_PULSE_MAX_US * cycles_per_us;
	const uint32_t rail_idle = rail_mv();
	uint32_t pulse = TPWR_PULSE_MIN_CYC;
	uint32_t pulses = 0;
	const uint32_t start_ms = platform_time_ms();

	while (platform_time_ms() - start_ms < TPWR_TIMEOUT_MS) {
		const uint32_t target = target_mv(rail_mv());
		if (target >= TPWR_DONE_MV) {
			gpio_clear(PWR_EN_PORT, PWR_EN_PIN);
			DEBUG_INFO("tpwr: %" PRIu32 "mV after %" PRIu32 " pulses, %" PRIu32 "ms, last %" PRIu32 " cycles\n",
				target, pulses, platform_time_ms() - start_ms, pulse);
			return true;
		}
		const uint32_t begin = DWT_CYCCNT;
		pwr_en_pulse(pulse);
		++pulses;
		const uint32_t rail = rail_mv();
		const uint32_t droop = rail_idle > rail ? rail_idle - rail : 0U;
		if (droop > TPWR_DROOP_MV)
			pulse = MAX((pulse * 3U) / 4U, TPWR_PULSE_MIN_CYC);
		else if (droop < TPWR_DROOP_MV / 2U)
			pulse = MIN(pulse + pulse / 8U + 1U, pulse_max);
		while (DWT_CYCCNT - begin < period)
			continue;
	}
	gpio_set(PWR_EN_PORT, PWR_EN_PIN);
	DEBUG_ERROR("tpwr: T_VDD %" PRIu32 "mV after %" PRIu32 "ms, last pulse %" PRIu32 " cycles\n",
		target_mv(rail_mv()), (uint32_t)TPWR_TIMEOUT_MS, pulse);
	return false;
}

bool platform_target_get_power(void)
{
	return !gpio_get(PWR_EN_PORT, PWR_EN_PIN);
}

bool platform_target_set_power(const bool power)
{
	if (!power) {
		gpio_set(PWR_EN_PORT, PWR_EN_PIN);
		return true;
	}
	if (platform_target_get_power())
		return true;
	/* Keep the target in reset while its supply ramps, as a reset supervisor would */
	platform_nrst_set_val(true);
	const bool result = target_power_soft_start();
	platform_delay(TPWR_NRST_HOLD_MS);
	platform_nrst_set_val(false);
	return result;
}
#endif

bool platform_spi_init(const spi_bus_e bus)
{
	uint32_t controller = 0;
	if (bus == SPI_BUS_EXTERNAL)
	{
		gpio_mode_setup(EXT_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI);
		gpio_mode_setup(EXT_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EXT_SPI_CS);
		gpio_set_output_options(
			EXT_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI | EXT_SPI_CS);
		gpio_set_af(EXT_SPI_PORT, GPIO_AF5, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI);
		// 	/* Deselect the targeted peripheral chip */
		gpio_set(EXT_SPI_PORT, EXT_SPI_CS);

		rcc_periph_clock_enable(RCC_SPI1);
		rcc_periph_reset_pulse(RST_SPI1);
		controller = EXT_SPI;
	}
	else
		return false;

	/* Set up hardware SPI: master, PCLK/8, Mode 0, 8-bit MSB first */
	spi_init_master(controller, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
	spi_enable(controller);
	return true;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
	if (bus == SPI_BUS_EXTERNAL)
	{
		spi_disable(EXT_SPI);
		rcc_periph_clock_disable(RCC_SPI1);
		gpio_mode_setup(
			EXT_SPI_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, EXT_SPI_SCLK | EXT_SPI_MISO | EXT_SPI_MOSI | EXT_SPI_CS);
		return true;
	}
	else
		return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	const uint8_t device = device_select & 0x7fU;
	const bool select = !(device_select & 0x80U);
	uint32_t port;
	uint16_t pin;
	switch (device)
	{
	case SPI_DEVICE_EXT_FLASH:
		port = EXT_SPI_CS_PORT;
		pin = EXT_SPI_CS;
		break;
	default:
		return false;
	}
	gpio_set_val(port, pin, select);
	return true;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	switch (bus)
	{
	case SPI_BUS_EXTERNAL:
		return spi_xfer(EXT_SPI, value);
		break;
	default:
		return 0U;
	}
}

void BTN_ISR(void)
{
	exti_reset_request(BTN_PIN);
	scb_reset_system();
}