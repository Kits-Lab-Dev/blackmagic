/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022-2023 1BitSquared  <info@1bitsquared.com>
 * Portions (C) 2020-2021 Stoyan Shopov <stoyan.shopov@gmail.com>
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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef PLATFORMS_STM32L432_PLATFORM_H
#define PLATFORMS_STM32L432_PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/usb/usbd.h>

// #define ENABLE_DEBUG 1
// #define PLATFORM_HAS_DEBUG
// extern bool debug_bmp;

#define PLATFORM_HAS_TRACESWO
#define SWO_ENCODING 2 /* Use only UART mode SWO recovery */

#define DFU_SERIAL_LENGTH 13

#define PLATFORM_IDENT "(STM32L432-IF) "

#define PLATFORM_HAS_POWER_SWITCH

/* Hardware definitions... */
#define TDI_PORT GPIOB
#define TMS_PORT GPIOB
#define TCK_PORT GPIOB
#define TDO_PORT GPIOB
#define TDI_PIN GPIO6
#define TMS_PIN GPIO4
#define TCK_PIN GPIO5
#define TDO_PIN GPIO7


#define SWDIO_PORT TMS_PORT
#define SWCLK_PORT TCK_PORT

#define SWDIO_PIN TMS_PIN
#define SWCLK_PIN TCK_PIN

#define TMS_DIR_PORT GPIOB
#define TMS_DIR_PIN GPIO0

#define SWDIO_DIR_PORT TMS_DIR_PORT
#define SWDIO_DIR_PIN TMS_DIR_PIN

#define NRST_PORT GPIOB
#define NRST_PIN GPIO1

#define SWO_PORT TDO_PORT
#define SWO_PIN TDO_PIN

#define PWR_EN_PORT GPIOA
#define PWR_EN_PIN GPIO15

#define TARGET_V_ADC ADC1
#define TARGET_V_PORT GPIOA
#define TARGET_V_PIN GPIO0
#define TARGET_V_CH 5                                       

#define SWDIO_MODE_FLOAT()                                 \
	do                                                     \
	{                                              \
		gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWDIO_PIN); \
		gpio_clear(TMS_DIR_PORT, TMS_DIR_PIN);         \
	} while (0)

#define SWDIO_MODE_DRIVE()                                  \
	do                                                      \
	{                                                         \
		gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN); \
		gpio_set_output_options(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, SWDIO_PIN); \
		gpio_set(TMS_DIR_PORT, TMS_DIR_PIN);                \
	} while (0)

#define TMS_SET_MODE() 

extern const struct _usbd_driver l432_usb_driver;
#define USB_DRIVER l432_usb_driver
#define USB_IRQ NVIC_OTG_FS_IRQ
#define USB_ISR(x) otg_fs_isr(x)

#undef USB_OTG_FS_BASE
#define USB_OTG_FS_BASE USB_DEV_FS_BASE

/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 */

#define IRQ_PRI_USB (1U << 6U)
#define IRQ_PRI_USBUSART (2U << 6U)
#define IRQ_PRI_USBUSART_DMA (2U << 6U)
#define IRQ_PRI_SWO_DMA (2U << 6U)

#define USBUSART USART2
#define USBUSART_BASE USART2_BASE

#define USBUSART_CR1 USART_CR1(USBUSART_BASE)
#define USBUSART_RDR USART_RDR(USBUSART_BASE)
#define USBUSART_TDR USART_TDR(USBUSART_BASE)
#define USBUSART_IRQ NVIC_USART2_IRQ
#define USBUSART_CLK RCC_USART2
#define USBUSART_PORT GPIOA
#define USBUSART_PIN_AF GPIO_AF7
#define USBUSART_PORT_CLKEN RCC_GPIOA
#define USBUSART_TX_PIN GPIO2
#define USBUSART_RX_PIN GPIO3
#define USBUSART_ISR usart2_isr

#define USBUSART_DMA_BUS DMA1
#define USBUSART_DMA_CLK RCC_DMA1
#define USBUSART_DMA_TX_CHAN DMA_CHANNEL7
#define USBUSART_DMA_TX_IRQ NVIC_DMA1_CHANNEL7_IRQ
#define USBUSART_DMA_TX_ISR(x) dma1_channel7_isr(x)
#define USBUSART_DMA_RX_CHAN DMA_CHANNEL6
#define USBUSART_DMA_RX_IRQ NVIC_DMA1_CHANNEL6_IRQ
#define USBUSART_DMA_RX_ISR(x) dma1_channel6_isr(x)

/* For STM32L4 DMA trigger source must be specified */
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4

#define UART_PIN_SETUP()                                                                                   \
	do                                                                                                     \
	{   /*usart2 dma request*/   																			\
		dma_set_channel_request(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, 2); 								\
		dma_set_channel_request(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, 2); 								\
																											\
		rcc_periph_clock_enable(USBUSART_PORT_CLKEN);                                                      \
		gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USBUSART_TX_PIN | USBUSART_RX_PIN); \
		gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_VERYHIGH, USBUSART_TX_PIN);      \
		gpio_set_af(USBUSART_PORT, USBUSART_PIN_AF, USBUSART_TX_PIN | USBUSART_RX_PIN);                    \
	} while (0)

#define SWO_UART USART1
#define SWO_UART_DR USART1_RDR
#define SWO_UART_CLK RCC_USART1
#define SWO_UART_PORT GPIOA
#define SWO_UART_RX_PIN GPIO10
#define SWO_UART_PIN_AF GPIO_AF7

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS DMA1
#define SWO_DMA_CLK RCC_DMA1
#define SWO_DMA_CHAN DMA_CHANNEL5
#define SWO_DMA_TRG DMA_SxCR_CHSEL_4
#define SWO_DMA_IRQ NVIC_DMA1_CHANNEL5_IRQ
#define SWO_DMA_ISR(x) dma1_channel5_isr(x)

#define LED_PORT GPIOB
#define LED_ERROR GPIO14
#define LED_PORT_ERROR GPIOA
#define LED_PORT_UART GPIOA
#define LED_UART GPIO13

#define LED_IDLE_RUN GPIO3
#define SET_RUN_STATE(state)      \
	{                             \
		running_status = (state); \
	}
#define SET_IDLE_STATE(state)                        \
	{                                                \
		gpio_set_val(LED_PORT, LED_IDLE_RUN, state); \
	}
#define SET_ERROR_STATE(state)     					 \
	{                                                \
		gpio_set_val(LED_PORT_ERROR, LED_ERROR, state); \
	}

#define BTN_PORT GPIOH
#define BTN_PIN GPIO3
#define BTN_IRQ NVIC_EXTI3_IRQ
#define BTN_ISR(x) exti3_isr(x)

#define EXT_SPI SPI1
#define EXT_SPI_PORT GPIOA
#define EXT_SPI_SCLK GPIO5
#define EXT_SPI_MISO GPIO6
#define EXT_SPI_MOSI GPIO7
#define EXT_SPI_CS_PORT GPIOA
#define EXT_SPI_CS GPIO4

extern uint32_t detect_rev(void);

#endif /* PLATFORMS_STM32L432_PLATFORM_H */
