/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015  Piotr Miedzik  <P.Miedzik@gsi.de>
 *   Copyright (C) 2015-2016  Henrique Silva <henrique.silva@lnls.br>
 *   Copyright (C) 2021  Krzysztof Macias <krzysztof.macias@creotech.pl>
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
 *
 *   @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

/**
 * @file afcv4/payload.h
 * @brief Payload control module definitions for AFCv4.0
 *
 * @ingroup AFC_V4_0_PAYLOAD
 */

/**
 * @defgroup AFC_V4_0_PAYLOAD AFCv4 Payload Control
 * @ingroup AFC_V4_0
 * @{
 */

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#include "event_groups.h"

typedef struct
{
  uint16_t address; /* 16-bit register address */
  uint8_t value; /* 8-bit register data */
} si5345_revd_register_t;

/**
 * @brief Payload state machine state numbers
 */
extern enum {
    PAYLOAD_NO_POWER = 0,
    PAYLOAD_POWER_GOOD_WAIT,
    PAYLOAD_STATE_CLK_SETUP,
    PAYLOAD_STATE_FPGA_SETUP,
    PAYLOAD_FPGA_ON,
    PAYLOAD_SWITCHING_OFF,
    PAYLOAD_QUIESCED,
    PAYLOAD_RESET,
    PAYLOAD_MAX_STATES,
} payload_state;

/**
 * @defgroup AFC_V4_0_PAYLOAD_MSG Payload Messages Codes
 * @ingroup AFC_V4_0_PAYLOAD
 * @{
 */
#define PAYLOAD_MESSAGE_COLD_RST        (1 << 0)
#define PAYLOAD_MESSAGE_WARM_RST        (1 << 1)
#define PAYLOAD_MESSAGE_REBOOT          (1 << 2)
#define PAYLOAD_MESSAGE_QUIESCE         (1 << 3)
#define PAYLOAD_MESSAGE_RTM_ENABLE      (1 << 4)
#define PAYLOAD_MESSAGE_DCDC_PGOOD      (1 << 5)
#define PAYLOAD_MESSAGE_DCDC_PGOODn     (1 << 6)
#define PAYLOAD_MESSAGE_CLOCK_CONFIG	  (1 << 7)
/**
 * @}
 */

/**
 * @brief Payload task unblock delay
 */
#define PAYLOAD_BASE_DELAY 1000

#ifdef MODULE_UCD90XXX

extern enum {
  UCD_AMC_12V_ID = 0,
  UCD_RTM_12V_CURR_ID,
  UCD_FMC0_12V_CURR_ID,
  UCD_FMC0_VADJ_ID,
  UCD_FMC0_3V3_CURR_ID,
  UCD_FMC1_12V_CURR_ID,
  UCD_FMC1_VADJ_ID,
  UCD_FMC1_3V3_CURR_ID,
  UCD_FPGA_0V85_ID,
  UCD_FPGA_1V8_ID,
  UCD_FPGA_0V9A_ID,
  UCD_FPGA_1V2A_ID,
  UCD_DDR_1V2T_ID,
  UCD_DDR_1V2B_ID,
  UCD_AMC_IO_3V3_ID,
  UCD_CH_COUNT
} UCD_AMC;

// Input
#define UCD_GPIO_PG               0     // MAR1

// #define UCD_GPIO_FMC0_PRSNT       9     // GPI2
// #define UCD_GPIO_FMC0_3V3AUX_FLT  25    // GPIO18
// #define UCD_GPIO_FMC0_3V3_FLT     23    // GPIO16
// #define UCD_GPIO_FMC0_12V0_FLT    12    // GPIO14

// #define UCD_GPIO_FMC1_PRSNT       8     // GPI1
// #define UCD_GPIO_FMC1_3V3AUX_FLT  24    // GPIO17
// #define UCD_GPIO_FMC1_3V3_FLT     19    // GPIO2
// #define UCD_GPIO_FMC1_12V0_FLT    21    // GPIO4

// // Output
// #define UCD_GPIO_FMC0_3V3AUX_EN   2     // GPIO7
// #define UCD_GPIO_FMC1_3V3AUX_EN   0     // GPIO5

// #define UCD_GPIO_FMC0_VADJ_PG     4     // GPIO9
// #define UCD_GPIO_FMC1_VADJ_PG     5     // GPIO10

// #define UCD_GPIO_FMC0_12V0_PG     18    // GPIO1
// #define UCD_GPIO_FMC1_12V0_PG     20    // GPIO3

// #define UCD_GPIO_FMC0_3V3_PG      22    // GPIO13
// #define UCD_GPIO_FMC1_3V3_PG      13    // GPIO15

#endif


/**
 * @brief Payload task handle variable
 */
extern TaskHandle_t vTaskPayload_Handle;

/**
 * @brief Sends a message to the payload task
 *
 * This function basically sets a flag that the Payload task reads and advances (or not) on the state machine
 *
 * @param fru_id Target FRU ID (0:AMC 1:RTM)
 * @param msg Message to send, using @ref AFC_V3_1_PAYLOAD_MSG definitions
 */
void payload_send_message( uint8_t fru_id, EventBits_t msg );

/**
 * @brief Payload Control task
 *
 * @param pvParameters Pointer to buffer holding parameters passed to task upon initialization
 */
void vTaskPayload( void *pvParameters );

/**
 * @brief Creates Payload Control task and initializes the board's needed hardware
 */
void payload_init( void );

#ifdef MODULE_HPM

#define PAYLOAD_HPM_PAGE_SIZE    256

uint8_t payload_hpm_prepare_comp( void );
uint8_t payload_hpm_upload_block( uint8_t * block, uint16_t size );
uint8_t payload_hpm_finish_upload( uint32_t image_size );
uint8_t payload_hpm_get_upgrade_status( void );
uint8_t payload_hpm_activate_firmware( void );
#endif

#endif /* IPMI_PAYLOAD_H_ */

/**
 * @}
 */
