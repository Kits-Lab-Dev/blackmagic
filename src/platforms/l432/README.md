# BMP firmware for STM32L432 (KitsLab Lobzik)

What this port changes compared to upstream Black Magic Debug: [KITSLAB.md](../../../KITSLAB.md).

## Connections

* TDI:             PB6 (input while SWDIO is turned around, shares DIR with SWDIO)
* TMS / SWDIO:     PB4
* TMS / SWDIO DIR: PB0
* TCK / SWCLK:     PB5
* TDO / SWO:       PB7 (SWO via USART1 RX)
* NRST:            PB1
* PWR_EN:          PA15 (active low, soft start)
* VTARGET:         PA0 (ADC1 channel 5)
* UART TX:         PA2 (USART2)
* UART RX:         PA3 (USART2)
* LED green:       PB3 (idle/run)
* LED yellow:      PA13 (UART)
* LED red:         PA14 (error)
* Button:          PH3 (BOOT0)
