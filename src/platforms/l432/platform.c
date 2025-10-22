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

static uint32_t hw_version = 100;

int platform_hwversion(void)
{
	return hw_version;
}

void platform_nrst_set_val(bool assert)
{
	gpio_set_val(TRST_PORT, TRST_PIN, assert);
	if (assert)
	{
		for (volatile size_t i = 0; i < 10000; i++)
			continue;
	}
}

bool platform_nrst_get_val()
{
	return gpio_get(TRST_PORT, TRST_PIN) != 0;
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";

	adc_set_regular_sequence(TARGET_V_ADC, 1, (uint8_t[]){TARGET_V_CH});
	adc_start_conversion_regular(TARGET_V_ADC);
	while (!adc_eos(TARGET_V_ADC))
		continue;
	uint32_t value = adc_read_regular(TARGET_V_ADC);

	value *= 3379;	 /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';

	return ret;
}

uint32_t platform_target_voltage_sense(void)
{
	adc_set_regular_sequence(TARGET_V_ADC, 1, (uint8_t[]){TARGET_V_CH});
	adc_start_conversion_regular(TARGET_V_ADC);
	while (!adc_eos(TARGET_V_ADC))
		continue;
	uint32_t value = adc_read_regular(TARGET_V_ADC);

	return (value * 99U) / 8191U;
}

void platform_request_boot(void)
{
	//FIXME
	//вход boot подтянуть к питанию через встроенный резистор


	scb_reset_system();
}

#define ADC_CR_BITS_PROPERTY_RS (ADC_CR_ADCAL | ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_JADSTART | ADC_CR_JADSTP | ADC_CR_ADSTART | ADC_CR_ADSTP)

void platform_init(void)
{
	SCB_VTOR = (uintptr_t)&vector_table;

	rcc_clock_setup_pll(&rcc_hsi16_configs[RCC_CLOCK_VRANGE1_80MHZ]);

	flash_dcache_enable();
	flash_icache_enable();
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

	adc_power_on(TARGET_V_ADC);
	for (int i = 0; i < 100000; i++)
		__asm__("nop");

	gpio_mode_setup(BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, BTN_PIN);
	exti_select_source(BTN_PIN, BTN_PORT);
	exti_set_trigger(BTN_PIN, EXTI_TRIGGER_RISING);
	nvic_enable_irq(BTN_IRQ);
	exti_enable_request(BTN_PIN);

	gpio_set_output_options(TRST_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_VERYHIGH, TRST_PIN);
	gpio_mode_setup(TRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TRST_PIN);
	gpio_set(TRST_PORT, TRST_PIN);

	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TMS_PIN);
	gpio_set_output_options(TMS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TMS_PIN);

	gpio_mode_setup(TDI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDI_PIN);
	gpio_set_output_options(TDI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, TDI_PIN);

	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, TDO_PIN);

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
bool platform_target_get_power(void)
{
	return !gpio_get(PWR_EN_PORT, PWR_EN_PIN);
}

bool platform_target_set_power(const bool power)
{
	gpio_set_val(PWR_EN_PORT, PWR_EN_PIN, !power);
	return true;
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