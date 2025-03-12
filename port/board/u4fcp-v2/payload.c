/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015  Piotr Miedzik  <P.Miedzik@gsi.de>
 *   Copyright (C) 2015-2016  Henrique Silva <henrique.silva@lnls.br>
 *   Copyright (C) 2021  Krzysztof Macias <krzysztof.macias@creotech.pl>
 *   Copyright (C) 2025  Jie Zhang <zhj@ihep.ac.cn>
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

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Project Includes */
#include "port.h"
#include "payload.h"
#include "ipmi.h"
#include "task_priorities.h"
#include "hotswap.h"
#include "utils.h"
#include "fru.h"
#include "led.h"
#include "board_led.h"
#include "board_config.h"
#include "eeprom_24xx512.h"
#include "i2c.h"
#include "i2c_mapping.h"
#ifdef MODULE_ADN4604
#include "adn4604.h"
#include "clock_config.h"
#include "default_0-Registers.h"
#include "default_1-Registers.h"
#endif
#ifdef MODULE_VIO_TPL0102
#include "tpl0102.h"
#endif
// #include "ad84xx.h"
// #include "mcp23016.h"

/* payload states
 *   0 - No power
 *
 *   1 - Power Good wait
 *       Enable DCDC Converters
 *       Hotswap backend power failure and shutdown status clear
 *
 *   2 - FPGA setup
 *       One-time configurations (clock switch - ADN4604)
 *
 *   3 - FPGA on
 *
 *   4 - Power switching off
 *       Disable DCDC Converters
 *       Send "quiesced" event if requested
 *
 *   5 - Power quiesced
 *       Payload was safely turned off
 *       Wait until payload power goes down to restart the cycle
 */

/**
 * @brief Set uFC's DCDC Converters state
 *
 * @param on DCDCs state
 *
 */
uint8_t setDC_DC_ConvertersON(bool on)
{
  if (on)
  {
#ifdef DEBUG
    printf("Enable Power\n");
#endif
    gpio_set_pin_high(PIN_PORT(GPIO_PMBUS_CTRL), PIN_NUMBER(GPIO_PMBUS_CTRL));
  }
  else
  {
#ifdef DEBUG
    printf("Disable Power\n");
#endif
    gpio_set_pin_low(PIN_PORT(GPIO_PMBUS_CTRL), PIN_NUMBER(GPIO_PMBUS_CTRL));
  }
  return 1;
}

static void fpga_soft_reset(void)
{
  gpio_set_pin_low(PIN_PORT(GPIO_FPGA_RESET_B), PIN_NUMBER(GPIO_FPGA_RESET_B));
  asm("NOP");
  gpio_set_pin_high(PIN_PORT(GPIO_FPGA_RESET_B), PIN_NUMBER(GPIO_FPGA_RESET_B));
#ifdef DEBUG
  printf("Reset FPGA\n");
#endif
  /* Blink RED LED to indicate to the user that the Reset was performed */
  LEDUpdate(FRU_AMC, LED1, LEDMODE_LAMPTEST, LEDINIT_ON, 5, 0);
}

uint8_t payload_check_pgood()
{
  /* Threshold set to ~8V */
  // const uint16_t PAYLOAD_THRESHOLD = 0x9B2;
  // uint16_t dataADC;

  // Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

  // /* Waiting for A/D conversion complete */
  // while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH1, ADC_DR_DONE_STAT) != SET) {}
  // /* Read ADC value */
  // Chip_ADC_ReadValue(LPC_ADC, ADC_CH1, &dataADC);

  // if (dataADC > PAYLOAD_THRESHOLD){
  //     return 1;
  // }
  // return 0;

  // sensor_t * p_sensor;
  // SDR_type_01h_t *sdr;

  // extern const SDR_type_01h_t SDR_FMC1_12V;

  // /* Iterate through the SDR Table to find all the LM75 entries */
  // for ( p_sensor = sdr_head; (p_sensor != NULL) || (p_sensor->task_handle == NULL); p_sensor = p_sensor->next) {
  //     if (p_sensor->sdr == &SDR_FMC1_12V) {
  //         sdr = ( SDR_type_01h_t * ) p_sensor->sdr;
  //         *pgood_flag = ( ( p_sensor->readout_value >= (sdr->lower_critical_thr ) ) &&
  //                         ( p_sensor->readout_value <= (sdr->upper_critical_thr ) ) );
  //         return 1;
  //     }
  // }

  // return 0;

  if (gpio_read_pin(PIN_PORT(GPIO_HOT_SWAP_HANDLE), PIN_NUMBER(GPIO_HOT_SWAP_HANDLE)))
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

#ifdef MODULE_VIO_TPL0102

// Vout = 0.6V * (60.4k/(30.1k + dac * 100k / 256)+1)
#define CONV_VLOT(voltage) (uint8_t)((60.4 / ((voltage) / 0.6 - 1) - 30.2) * 256 / 100)
#define CONV_DAC(dac) (float)(0.6 * (60.4 / (30.2 + (dac) * 100 / 256) + 1))

void tpl0102_set_voltage(uint8_t chn, float voltage, bool non_volatile)
{
  uint8_t dac;
  if (voltage > 1.8)
    voltage = 1.8;
  if (voltage < 0.9)
    voltage = 0.9;

  dac = CONV_VLOT(voltage);
#ifdef DEBUG
  printf("Set FMC%d DAC=%d\n", chn, dac);
#endif
  if (non_volatile)
    tpl0102_set_non_volatile_val(chn, dac);
  else
    tpl0102_set_val(chn, dac);
}

float tpl0102_get_voltage(uint8_t chn)
{
  uint8_t val;
  float voltage;

  val = tpl0102_get_val(chn);
#ifdef DEBUG
  printf("Get DAC=%d\n", val);
#endif
  voltage = CONV_DAC(val);
  return voltage;
}

#endif

#ifdef MODULE_ADN4604

mmc_err pll_configuration(uint8_t chip_id, const si5345_revd_register_t* si5345_revd_registers_ptr)
{
  uint8_t page_curr, page_prev = 0xFF;
  uint32_t i;
  uint8_t i2c_addr, i2c_interface;
  uint8_t i2c_written = 0;
  uint8_t err_cnt = 0;
  uint8_t tx_data[2];

	for(i = 0; i < SI5345_REVD_REG_CONFIG_NUM_REGS; i++)
	{
    if(i == 3) vTaskDelay(pdMS_TO_TICKS(300));
		page_curr = (uint8_t)((si5345_revd_registers_ptr[i].address>>8)&0xFF);
    tx_data[0] = 0x1;
    tx_data[1] = page_curr;
		if (page_curr != page_prev)
    {
      err_cnt = 0;
      i2c_written = 0;
      while (i2c_written != 2)
      {
        if (i2c_take_by_chipid(chip_id, &i2c_addr, &i2c_interface, portMAX_DELAY) == pdTRUE)
        {
          i2c_written = xI2CMasterWrite(i2c_interface, i2c_addr, tx_data, 2);
          i2c_give(i2c_interface);
        }
        if (i2c_written!=2)
        {
          err_cnt++;
          if(err_cnt == 0xF) return MMC_IO_ERR;
          else vTaskDelay(pdMS_TO_TICKS(5)); /* Avoid too much unnecessary I2C trafic*/
        }
        // else printf("PLL write 0x%x to 0x%x\n", tx_data[1], tx_data[0]);
      }
    }
    tx_data[0] = (uint8_t)(si5345_revd_registers_ptr[i].address&0xFF);
    tx_data[1] = si5345_revd_registers_ptr[i].value;
    err_cnt = 0;
    i2c_written = 0;
    while (i2c_written != 2)
    {
      if (i2c_take_by_chipid(chip_id, &i2c_addr, &i2c_interface, portMAX_DELAY) == pdTRUE)
      {
        i2c_written = xI2CMasterWrite(i2c_interface, i2c_addr, tx_data, 2);
        i2c_give(i2c_interface);
      }
      if (i2c_written!=2)
      {
        err_cnt++;
        if(err_cnt == 0xF) return MMC_IO_ERR;
        else vTaskDelay(pdMS_TO_TICKS(5)); /* Avoid too much unnecessary I2C trafic*/
      }
      // else printf("PLL write 0x%x to 0x%x\n", tx_data[1], tx_data[0]);
    }
    page_prev = page_curr;
	}
  return MMC_OK;
}

mmc_err clock_configuration(const uint8_t clk_cfg[16])
{
  adn_connect_map_t con;
  mmc_err error;

  /* Translate the configuration to enable or disable the outputs */
  uint16_t out_enable_flag = {
      ((clk_cfg[0] & 0x80) >> 7) << 0 |
      ((clk_cfg[1] & 0x80) >> 7) << 1 |
      ((clk_cfg[2] & 0x80) >> 7) << 2 |
      ((clk_cfg[3] & 0x80) >> 7) << 3 |
      ((clk_cfg[4] & 0x80) >> 7) << 4 |
      ((clk_cfg[5] & 0x80) >> 7) << 5 |
      ((clk_cfg[6] & 0x80) >> 7) << 6 |
      ((clk_cfg[7] & 0x80) >> 7) << 7 |
      ((clk_cfg[8] & 0x80) >> 7) << 8 |
      ((clk_cfg[9] & 0x80) >> 7) << 9 |
      ((clk_cfg[10] & 0x80) >> 7) << 10 |
      ((clk_cfg[11] & 0x80) >> 7) << 11 |
      ((clk_cfg[12] & 0x80) >> 7) << 12 |
      ((clk_cfg[13] & 0x80) >> 7) << 13 |
      ((clk_cfg[14] & 0x80) >> 7) << 14 |
      ((clk_cfg[15] & 0x80) >> 7) << 15};

  /* Disable UPDATE' pin by pulling it GPIO_LEVEL_HIGH */
  // gpio_set_pin_state(PIN_PORT(GPIO_ADN_UPDATE), PIN_NUMBER(GPIO_ADN_UPDATE), GPIO_LEVEL_HIGH);

  /* There's a delay circuit in the Reset pin of the clock switch, we must wait until it clears out */
  // while (gpio_read_pin(PIN_PORT(GPIO_ADN_RESETN), PIN_NUMBER(GPIO_ADN_RESETN)) == 0)
  // {
  //   vTaskDelay(50);
  // }

  /* Configure the interconnects*/
  con.out0 = clk_cfg[0] & 0x0F;
  con.out1 = clk_cfg[1] & 0x0F;
  con.out2 = clk_cfg[2] & 0x0F;
  con.out3 = clk_cfg[3] & 0x0F;
  con.out4 = clk_cfg[4] & 0x0F;
  con.out5 = clk_cfg[5] & 0x0F;
  con.out6 = clk_cfg[6] & 0x0F;
  con.out7 = clk_cfg[7] & 0x0F;
  con.out8 = clk_cfg[8] & 0x0F;
  con.out9 = clk_cfg[9] & 0x0F;
  con.out10 = clk_cfg[10] & 0x0F;
  con.out11 = clk_cfg[11] & 0x0F;
  con.out12 = clk_cfg[12] & 0x0F;
  con.out13 = clk_cfg[13] & 0x0F;
  con.out14 = clk_cfg[14] & 0x0F;
  con.out15 = clk_cfg[15] & 0x0F;

  error = adn4604_xpt_config(ADN_XPT_MAP0_CON_REG, con);
  if (error != MMC_OK)
  {
    return error;
  }

  /* Enable desired outputs */
  for (uint8_t i = 0; i < 16; i++)
  {
    if ((out_enable_flag >> i) & 0x1)
    {
      error = adn4604_tx_control(i, TX_ENABLED);
      if (error != MMC_OK)
      {
        return error;
      }
    }
    else
    {
      error = adn4604_tx_control(i, TX_DISABLED);
      if (error != MMC_OK)
      {
        return error;
      }
    }
  }

  error = adn4604_active_map(ADN_XPT_MAP0);
  if (error != MMC_OK)
  {
    return error;
  }

  return adn4604_update();
}
#endif

EventGroupHandle_t amc_payload_evt = NULL;
#ifdef MODULE_RTM
EventGroupHandle_t rtm_payload_evt = NULL;
#endif

void payload_send_message(uint8_t fru_id, EventBits_t msg)
{
  if ((fru_id == FRU_AMC) && amc_payload_evt)
  {
    xEventGroupSetBits(amc_payload_evt, msg);
#ifdef MODULE_RTM
  }
  else if ((fru_id == FRU_RTM) && rtm_payload_evt)
  {
    xEventGroupSetBits(rtm_payload_evt, msg);
#endif
  }
}

TaskHandle_t vTaskPayload_Handle;

void payload_init(void)
{
  xTaskCreate(vTaskPayload, "Payload", 256, NULL, tskPAYLOAD_PRIORITY, &vTaskPayload_Handle);

  amc_payload_evt = xEventGroupCreate();
#ifdef MODULE_RTM
  rtm_payload_evt = xEventGroupCreate();
  mcp23016_write_pin(ext_gpios[EXT_GPIO_EN_RTM_MP].port_num, ext_gpios[EXT_GPIO_EN_RTM_MP].pin_num, true);
#endif

#ifdef MODULE_ADC
  ADC_CLOCK_SETUP_T ADCSetup;
  Chip_ADC_Init(LPC_ADC, &ADCSetup);
  Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);
#endif

#ifdef MODULE_VIO_TPL0102
  /* Configure the PVADJ DAC */
  tpl0102_init();
  tpl0102_set_voltage(0, 1.8, false);
  tpl0102_set_voltage(1, 1.8, false);
#endif
}

// Inputs for ADN4604
#define ADN4604_IN_AMC_TX_P17 0
#define ADN4604_IN_AMC_RX_P17 1
#define ADN4604_IN_TCLKD 2
#define ADN4604_IN_TCLKC 3
#define ADN4604_IN_TCLKA 4
#define ADN4604_IN_TCLKB 5
#define ADN4604_IN_FCLKA 6
#define ADN4604_IN_FPGA_CLK_OUT 7
#define ADN4604_IN_PLL1_OUT 8
#define ADN4604_IN_PLL0_OUT 9
#define ADN4604_IN_FMC1_CLK_M2C 10
#define ADN4604_IN_FMC1_CLK_BIDIR 11
#define ADN4604_IN_FMC0_CLK_M2C 12
#define ADN4604_IN_FMC0_CLK_BIDIR 13
#define ADN4604_IN_LEMO_CLK 14
#define ADN4604_IN_RTM2AMC_CLK 15

#define CLK_EN 0x80

void vTaskPayload(void *pvParameters)
{
  uint8_t clock_config[16];
  clock_config[0] = ADN4604_IN_PLL0_OUT;
  clock_config[1] = ADN4604_IN_PLL0_OUT | CLK_EN;
  clock_config[2] = ADN4604_IN_PLL0_OUT;
  clock_config[3] = ADN4604_IN_PLL0_OUT;
  clock_config[4] = ADN4604_IN_PLL0_OUT;
  clock_config[5] = ADN4604_IN_PLL0_OUT;
  clock_config[6] = ADN4604_IN_PLL0_OUT;
  clock_config[7] = ADN4604_IN_PLL0_OUT;
  clock_config[8] = ADN4604_IN_PLL0_OUT;
  clock_config[9] = ADN4604_IN_PLL0_OUT;
  clock_config[10] = ADN4604_IN_PLL0_OUT;
  clock_config[11] = ADN4604_IN_PLL0_OUT;
  clock_config[12] = ADN4604_IN_PLL0_OUT;
  clock_config[13] = ADN4604_IN_PLL0_OUT;
  clock_config[14] = ADN4604_IN_PLL0_OUT;
  clock_config[15] = ADN4604_IN_PLL0_OUT;

  uint8_t state = PAYLOAD_NO_POWER;
  /* Use arbitrary state value to force the first state update */
  uint8_t new_state = -1;

  /* Payload power good flag */
  uint8_t PP_good = 0;

  /* Payload DCDCs good flag */
  uint8_t DCDC_good = 0;

  /* FPGA program done flag */
  uint8_t FPGA_prom_done = 0;

  uint8_t QUIESCED_req = 0;
  EventBits_t current_evt;

  extern sensor_t *hotswap_amc_sensor;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

#ifdef MODULE_ADN4604
  /* Recover clock switch configuration saved in EEPROM */
  // eeprom_24xx512_read(CHIP_ID_RTC_EEPROM, 0x0, clock_config, 16, 10);
#endif

  if (get_ipmb_addr() == IPMB_ADDR_DISCONNECTED) // standalone mode
  {
    while (!DCDC_good)
    {
      DCDC_good = gpio_read_pin(PIN_PORT(GPIO_TP), PIN_NUMBER(GPIO_TP));
    }
#ifdef MODULE_ADN4604
    /* Recover clock switch configuration saved in EEPROM */
    // eeprom_24xx512_read(CHIP_ID_RTC_EEPROM, 0x0, clock_config, 16, 10);
    if (adn4604_reset() == MMC_OK)
      printf("ADN4604 reset OK\n");
    else
      printf("ADN4604 reset ERROR\n");
    if (clock_configuration(clock_config) == MMC_OK)
      printf("ADN4604 config OK\n");
    else
      printf("ADN4604 config ERROR\n");
    if(pll_configuration(CHIP_ID_SI5345_0, si5345_revd_registers_0) == MMC_OK)
      printf("PLL0 config OK\n");
    else
      printf("PLL0 config ERROR\n");
    if(pll_configuration(CHIP_ID_SI5345_1, si5345_revd_registers_1) == MMC_OK)
      printf("PLL1 config OK\n");
    else
      printf("PLL1 config ERROR\n");
#endif
    while (!FPGA_prom_done)
    {
      FPGA_prom_done = gpio_read_pin(PIN_PORT(GPIO_FPGA_DONE), PIN_NUMBER(GPIO_FPGA_DONE));
    }
    // reset FPGA
    fpga_soft_reset();

    for (;;)
    {
      vTaskDelayUntil(&xLastWakeTime, PAYLOAD_BASE_DELAY);
    }
  }

  for (;;)
  {
    new_state = state;

    current_evt = xEventGroupGetBits(amc_payload_evt);

    /*
     * When receive a PAYLOAD_MESSAGE_CLOCK_CONFIG message, configure the clock switch
     * and write the new configuration in EEPROM
     */
    if (current_evt & PAYLOAD_MESSAGE_CLOCK_CONFIG)
    {
#ifdef MODULE_ADN4604
      // eeprom_24xx02_write(CHIP_ID_RTC_EEPROM, 0x0, clock_config, 16, 10);
      if (PAYLOAD_FPGA_ON)
      {
        if (adn4604_reset() == MMC_OK)
          printf("ADN4604 reset OK\n");
        else
          printf("ADN4604 reset ERROR\n");
        if (clock_configuration(clock_config) == MMC_OK)
          printf("ADN4604 config OK\n");
        else
          printf("ADN4604 config ERROR\n");
        if(pll_configuration(CHIP_ID_SI5345_0, si5345_revd_registers_0) == MMC_OK)
          printf("PLL0 config OK\n");
        else
          printf("PLL0 config ERROR\n");
        if(pll_configuration(CHIP_ID_SI5345_1, si5345_revd_registers_1) == MMC_OK)
          printf("PLL1 config OK\n");
        else
          printf("PLL1 config ERROR\n");
      }
#endif
      xEventGroupClearBits(amc_payload_evt, PAYLOAD_MESSAGE_CLOCK_CONFIG);
    }
    if (current_evt & PAYLOAD_MESSAGE_COLD_RST)
    {
      state = PAYLOAD_RESET;
      xEventGroupClearBits(amc_payload_evt, PAYLOAD_MESSAGE_COLD_RST);
    }

    if ((current_evt & PAYLOAD_MESSAGE_REBOOT) || (current_evt & PAYLOAD_MESSAGE_WARM_RST))
    {
      fpga_soft_reset();
      xEventGroupClearBits(amc_payload_evt, PAYLOAD_MESSAGE_REBOOT | PAYLOAD_MESSAGE_WARM_RST);
    }

    if (current_evt & PAYLOAD_MESSAGE_QUIESCE)
    {
      /*
       * If you issue a shutdown fru command in the MCH shell, the payload power
       * task will receive a PAYLOAD_MESSAGE_QUIESCE message and set the
       * QUIESCED_req flag to '1' and the MCH will shutdown the 12VP0 power,
  {     * maki}ng the payload power task go to PAYLOAD_NO_POWER state.
       * So, if we are in the PAYLOAD_QUIESCED state and receive a
       * PAYLOAD_MESSAGE_QUIESCE message, the QUIESCED_req flag
       * should be '0'
       */
      // if (state == PAYLOAD_QUIESCED)
      //   QUIESCED_req = 0;
      // else
      QUIESCED_req = 1;
      xEventGroupClearBits(amc_payload_evt, PAYLOAD_MESSAGE_QUIESCE);
    }

    PP_good = payload_check_pgood();
    DCDC_good = gpio_read_pin(PIN_PORT(GPIO_TP), PIN_NUMBER(GPIO_TP));

    switch (state)
    {

    case PAYLOAD_NO_POWER:
      if (PP_good)
      {
        new_state = PAYLOAD_POWER_GOOD_WAIT;
      }
      break;

    case PAYLOAD_POWER_GOOD_WAIT:
      /* Turn DCDC converters on */
      setDC_DC_ConvertersON(true);

      /* Clear hotswap sensor backend power failure bits */
      hotswap_clear_mask_bit(HOTSWAP_AMC, HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK);
      hotswap_clear_mask_bit(HOTSWAP_AMC, HOTSWAP_BACKEND_PWR_FAILURE_MASK);

#ifdef DEBUG
      printf("DCDC_good=%d\n", DCDC_good);
#endif
      if (QUIESCED_req || (PP_good == 0))
      {
        new_state = PAYLOAD_SWITCHING_OFF;
      }
      else if (DCDC_good == 1)
      {
        new_state = PAYLOAD_STATE_FPGA_SETUP;
      }
      break;

    case PAYLOAD_STATE_FPGA_SETUP:
      // gpio_set_pin_state(PIN_PORT(GPIO_FMC1_PG_C2M), PIN_NUMBER(GPIO_FMC1_PG_C2M), GPIO_LEVEL_HIGH);
      // gpio_set_pin_state(PIN_PORT(GPIO_FMC2_PG_C2M), PIN_NUMBER(GPIO_FMC2_PG_C2M), GPIO_LEVEL_HIGH);

#ifdef MODULE_ADN4604
      /* Configure clock switch */
      if (adn4604_reset() == MMC_OK)
      {
        if (clock_configuration(clock_config) == MMC_OK)
        {
          printf("ADN4604 config OK\n");
          if(pll_configuration(CHIP_ID_SI5345_0, si5345_revd_registers_0) == MMC_OK)
          {
            printf("PLL0 config OK\n");
            if(pll_configuration(CHIP_ID_SI5345_1, si5345_revd_registers_1) == MMC_OK)
            {
              printf("PLL1 config OK\n");
              // Only change the state if the clock switch configuration succeeds
              new_state = PAYLOAD_FPGA_ON;
            }
            else printf("PLL1 config ERROR\n");
          }
          else printf("PLL0 config ERROR\n");
        }
        else printf("ADN4604 config ERROR\n");
      }
      // else printf("ADN4604 reset ERROR\n");
#else
      new_state = PAYLOAD_FPGA_ON;
#endif
      break;

    case PAYLOAD_FPGA_ON:
      if (QUIESCED_req == 1 || PP_good == 0 || DCDC_good == 0)
      {
        new_state = PAYLOAD_SWITCHING_OFF;
      }
      break;

    case PAYLOAD_SWITCHING_OFF:
      // gpio_set_pin_state(PIN_PORT(GPIO_FMC1_PG_C2M), PIN_NUMBER(GPIO_FMC1_PG_C2M), GPIO_LEVEL_LOW);
      // gpio_set_pin_state(PIN_PORT(GPIO_FMC2_PG_C2M), PIN_NUMBER(GPIO_FMC2_PG_C2M), GPIO_LEVEL_LOW);

      setDC_DC_ConvertersON(false);

      /* Respond to quiesce event if any */
      if (QUIESCED_req)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        hotswap_set_mask_bit(HOTSWAP_AMC, HOTSWAP_QUIESCED_MASK);
        hotswap_send_event(hotswap_amc_sensor, HOTSWAP_STATE_QUIESCED);
        hotswap_clear_mask_bit(HOTSWAP_AMC, HOTSWAP_QUIESCED_MASK);
        QUIESCED_req = 0;
      }
      new_state = PAYLOAD_QUIESCED;
      break;

    case PAYLOAD_QUIESCED:
      /* Wait until power goes down to restart the cycle */
      if (PP_good == 0 && DCDC_good == 0)
      {
        new_state = PAYLOAD_NO_POWER;
      }
      break;

    case PAYLOAD_RESET:
      /*Reset DCDC converters*/
      setDC_DC_ConvertersON(false);
      new_state = PAYLOAD_NO_POWER;
      break;

    default:
      break;
    }

    state = new_state;
    vTaskDelayUntil(&xLastWakeTime, PAYLOAD_BASE_DELAY);
  }
}

/* HPM Functions */
#ifdef MODULE_HPM

#include "flash_spi.h"
#include "string.h"

uint8_t *hpm_page = NULL;
uint8_t hpm_pg_index;
uint32_t hpm_page_addr;

uint8_t payload_hpm_prepare_comp(void)
{
  /* Initialize variables */
  if (hpm_page != NULL)
  {
    vPortFree(hpm_page);
  }

  hpm_page = (uint8_t *)pvPortMalloc(PAYLOAD_HPM_PAGE_SIZE);

  if (hpm_page == NULL)
  {
    /* Malloc failed */
    return IPMI_CC_OUT_OF_SPACE;
  }

  memset(hpm_page, 0xFF, PAYLOAD_HPM_PAGE_SIZE);

  hpm_pg_index = 0;
  hpm_page_addr = 0;

  /* Initialize flash */
  ssp_init(FLASH_SPI, FLASH_SPI_BITRATE, FLASH_SPI_FRAME_SIZE, SSP_MASTER, SSP_INTERRUPT);

  /* Prevent the FPGA from accessing the Flash to configure itself now */
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_HIGH);
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_LOW);
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_HIGH);
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_LOW);

  /* Erase FLASH */
  flash_bulk_erase();

  return IPMI_CC_COMMAND_IN_PROGRESS;
}

uint8_t payload_hpm_upload_block(uint8_t *block, uint16_t size)
{
  /* TODO: Check DONE pin before accessing the SPI bus, since the FPGA may be reading it in order to boot */
  uint8_t remaining_bytes_start;

  if (PAYLOAD_HPM_PAGE_SIZE - hpm_pg_index > size)
  {
    /* Our page is not full yet, just append the new data */
    memcpy(&hpm_page[hpm_pg_index], block, size);
    hpm_pg_index += size;

    return IPMI_CC_OK;
  }
  else
  {
    /* Complete the remaining bytes on the buffer */
    memcpy(&hpm_page[hpm_pg_index], block, (PAYLOAD_HPM_PAGE_SIZE - hpm_pg_index));
    remaining_bytes_start = (PAYLOAD_HPM_PAGE_SIZE - hpm_pg_index);

    /* Program the complete page in the Flash */
    flash_program_page(hpm_page_addr, &hpm_page[0], PAYLOAD_HPM_PAGE_SIZE);

    hpm_page_addr += PAYLOAD_HPM_PAGE_SIZE;

    /* Empty our buffer and reset the index */
    memset(hpm_page, 0xFF, PAYLOAD_HPM_PAGE_SIZE);
    hpm_pg_index = 0;

    /* Save the trailing bytes */
    memcpy(&hpm_page[hpm_pg_index], block + remaining_bytes_start, size - remaining_bytes_start);

    hpm_pg_index = size - remaining_bytes_start;

    return IPMI_CC_COMMAND_IN_PROGRESS;
  }
}

uint8_t payload_hpm_finish_upload(uint32_t image_size)
{
  uint8_t cc = IPMI_CC_OK;

  /* Check if the last page was already programmed */
  if (!hpm_pg_index)
  {
    /* Program the complete page in the Flash */
    flash_program_page(hpm_page_addr, &hpm_page[0], (PAYLOAD_HPM_PAGE_SIZE - hpm_pg_index));
    hpm_pg_index = 0;
    hpm_page_addr = 0;

    cc = IPMI_CC_COMMAND_IN_PROGRESS;
  }

  /* Free page buffer */
  vPortFree(hpm_page);
  hpm_page = NULL;

  return cc;
}

uint8_t payload_hpm_get_upgrade_status(void)
{
  if (is_flash_busy())
  {
    return IPMI_CC_COMMAND_IN_PROGRESS;
  }
  else
  {
    return IPMI_CC_OK;
  }
}

uint8_t payload_hpm_activate_firmware(void)
{
  /* Reset FPGA - Pulse PROGRAM_B pin */
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_LOW);
  gpio_set_pin_state(PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_HIGH);

  return IPMI_CC_OK;
}
#endif
