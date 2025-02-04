/*
 *   openMMC  --
 *
 *   Copyright (C) 2015  Henrique Silva  <henrique.silva@lnls.br>
 *   Copyright (C) 2025  Jie Zhang  <zhj@ihep.ac.cn>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file stm32_uart.h
 * @author Henrique Silva <henrique.silva@lnls.br>, LNLS
 * @author Jie Zhang  <zhj@ihep.ac.cn>, IHEP
 *
 * @brief STM32xx UART interface definitions
 */

#ifndef STM32_UART_H_
#define STM32_UART_H_

#include "string.h"
#include "stdio.h"

// typedef struct lpc_uart_cfg {
//     LPC_USART_T * ptr;
//     IRQn_Type irq;
//     CHIP_SYSCTL_PCLK_T sysclk;
// } lpc_uart_cfg_t;

// extern const lpc_uart_cfg_t usart_cfg[4];

// #define uart_set_baud( id, baud ) Chip_UART_SetBaud( usart_cfg[id].ptr, baud )
// #define uart_config_data( id, cfg ) Chip_UART_ConfigData( usart_cfg[id].ptr, cfg )
// #define uart_tx_enable( id ) Chip_UART_TXEnable( usart_cfg[id].ptr )
// #define uart_tx_disable( id ) Chip_UART_TXDisable( usart_cfg[id].ptr )
// #define uart_int_enable( id, mask ) Chip_UART_IntEnable( usart_cfg[id].ptr, mask )
// #define uart_int_disable( id, mask ) Chip_UART_IntDisable( usart_cfg[id].ptr, mask )
// #define uart_send_char( id, ch ) Chip_UART_SendByte( usart_cfg[id].ptr, ch )
// #define uart_read_char( id ) Chip_UART_ReadByte( usart_cfg[id].ptr )
// #define uart_send( id, msg, len ) Chip_UART_SendBlocking( usart_cfg[id].ptr, msg, len )
#define uart_send( id, msg, len ) 1
// #define uart_read( id, buf, len ) Chip_UART_ReadBlocking( usart_cfg[id].ptr, buf, len )

// void uart_init ( uint8_t id );

#endif
