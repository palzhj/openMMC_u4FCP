/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015-2016  Henrique Silva <henrique.silva@lnls.br>
 *   Copyright (C) 2025-2026  Jie Zhang <zhj@ihep.ac.cn>
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
 * @file ucd90xxx.c
 *
 * @brief ucd90xxx interface implementation
 *
 * @ingroup ucd90xxx
 */

/* Project Includes */
#include "port.h"
#include "ucd90xxx.h"
#include "i2c.h"
#include "i2c_mapping.h"
#include "task.h"
#include "task_priorities.h"
#include "math.h"
#include "payload.h"

/**
 * @defgroup UCD90xxx_REGS UCD90xxx Registers
 * @ingroup UCD90xxx
 * @{
 */
#define UCD_TIMEOUT 40 // ms, device times out when any clock low exceeds

#define UCD_REG_PAGE 0x00
#define UCD_REG_OPERATION 0x01
#define UCD_REG_VOUT_MODE 0x20
#define UCD_REG_STATUS_VOUT 0x7A
#define UCD_REG_STATUS_IOUT 0x7B
#define UCD_REG_STATUS_TEMPERATURE 0x7D
#define UCD_REG_READ_VOUT 0x8B
#define UCD_REG_READ_IOUT 0x8C
#define UCD_REG_READ_TEMPERATURE_1 0x8D
#define UCD_REG_READ_TEMPERATURE_2 0x8E
#define UCD_REG_GPIO_SELECT 0xFA
#define UCD_REG_GPIO_CONFIG 0xFB

/**
 * @}
 */

/* Status */
#define UCD_VOUT_OF 0x80 // Over voltage Fault
#define UCD_VOUT_OW 0x40 // Over voltage Warning
#define UCD_VOUT_UW 0x20 // Under voltage Warning
#define UCD_VOUT_UF 0x10 // Under voltage Fault

#define UCD_IOUT_OF 0x80    // Over current Fault
#define UCD_IOUT_OLVSF 0x40 // Over current and low voltage shutdown fault
#define UCD_IOUT_OW 0x20    // Over current Warning
#define UCD_IOUT_UF 0x10    // Under current Fault

#define UCD_T_OF 0x80 // Over temperature Fault
#define UCD_T_OW 0x40 // Over temperature Warning

#define UCD_GPIO_CONFIG_EN 0x01
#define UCD_GPIO_CONFIG_OEN 0x02
#define UCD_GPIO_CONFIG_HIGH 0x04
#define UCD_GPIO_CONFIG_STATUS 0x08

#define UCD_UPDATE_RATE 5000

static ucd_data_t ucd_data[SDR_CH_COUNT];
TaskHandle_t vTaskUCD_Handle;

/**
 * @brief Task function for handling UCD90xxx sensor updates.
 *
 * This task periodically reads data from UCD90xxx sensors and checks for threshold events.
 * It iterates through the available UCD90xxx channels, reads the sensor data based on the
 * sensor type (voltage or current), and performs necessary checks for sensor events.
 *
 * The task runs at a fixed interval defined by `UCD_UPDATE_RATE` and uses FreeRTOS timing
 * functions to maintain the periodicity.
 *
 * @param Parameters Pointer to task parameters (unused in this implementation).
 */
void vTaskUCD(void *Parameters)
{
  uint8_t ch_num;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(UCD_UPDATE_RATE);

  sensor_t *ucd_sensor;
  ucd_data_t *data_ptr;

  /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    /* Read registers from the UCD90xxx */
    for (ch_num = 0; ch_num < SDR_CH_COUNT; ch_num++)
    {
      /* Skip if the chip ID is not set */
      if (!ucd_data[ch_num].chipid)
      {
        continue;
      }

      ucd_sensor = ucd_data[ch_num].sensor;
      if (ucd_sensor == NULL)
      {
        continue;
      }

      data_ptr = &ucd_data[ch_num];

      /* Handle different sensor types */
      switch (GET_SENSOR_TYPE(ucd_sensor))
      {
      case SENSOR_TYPE_VOLTAGE:
        ucd_read_voltages(data_ptr);
        break;

      case SENSOR_TYPE_CURRENT:
        ucd_read_current(data_ptr);
        break;

      case SENSOR_TYPE_TEMPERATURE:
        ucd_read_temperature(data_ptr);
        break;

      default:
        break;
      }

      /* Check for threshold events */
      sensor_state_check(ucd_sensor);
      check_sensor_event(ucd_sensor);
    }

    /* Wait until the next cycle */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief Reads the status of a GPIO pin on a UCD90xxx device.
 *
 * This function selects the specified GPIO pin on the UCD90xxx device and then reads its
 * configuration register to determine the status of the pin. The function retries the I2C
 * operations until they succeed, with a delay between retries to avoid excessive I2C traffic.
 *
 * @param chipid The chip ID of the UCD90xxx device.
 * @param pin_number The GPIO pin number to read.
 *
 * @return 1 if the GPIO status is high, 0 if low.
 */
uint8_t ucd_get_gpio(uint8_t chipid, uint8_t pin_number)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t val[2] = {0};
  uint8_t len = 0;

  val[0] = UCD_REG_GPIO_SELECT;
  while (len != 1)
  {
    if (i2c_take_by_chipid(chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      len = xI2CMasterWriteRead(i2c_interf, i2c_addr, &val[0], 1, &val[1], 1);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    if (len != 1)
    {
#ifdef DEBUG
      printf("UCD read error @ 0x%x\n", UCD_REG_GPIO_SELECT);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }
  if (val[1] != pin_number)
  {
    /* Select the GPIO pin */
    val[1] = pin_number;
    len = 0;
    while (len != 2)
    {
      if (i2c_take_by_chipid(chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
      {
        len = xI2CMasterWrite(i2c_interf, i2c_addr, val, 2);
        i2c_give(i2c_interf); /* Release the I2C interface */
      }

      if (len != 2)
      {
#ifdef DEBUG
        printf("UCD write error @ 0x%x\n", UCD_REG_GPIO_SELECT);
#endif
        vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
      }
    }
    vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT));
  }

  /* Read the GPIO configuration register */
  val[0] = UCD_REG_GPIO_CONFIG;
  len = 0;
  while (len != 1)
  {
    if (i2c_take_by_chipid(chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      len = xI2CMasterWriteRead(i2c_interf, i2c_addr, &val[0], 1, &val[1], 1);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    if (len != 1)
    {
#ifdef DEBUG
      printf("UCD read error @ 0x%x\n", UCD_REG_GPIO_CONFIG);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }

  /* Check the GPIO status bit */
  return (val[1] & UCD_GPIO_CONFIG_STATUS) ? 1 : 0;
}

/**
 * @brief Reads a single byte from a UCD90xxx device register.
 *
 * This function reads a single byte from the specified register address of a UCD90xxx device.
 * It uses the I2C interface to communicate with the device. The function will retry the read
 * operation until it successfully reads one byte of data. If the read operation fails, it will
 * delay for a short period to avoid excessive I2C traffic and then retry.
 *
 * @param data Pointer to the UCD90xxx data structure containing the chip ID and other relevant information.
 * @param reg_addr The register address from which to read the byte.
 * @param read Pointer to a variable where the read byte will be stored.
 *
 * @return The number of bytes successfully read (1 if successful).
 */
uint8_t ucd_read_byte(ucd_data_t *data, uint8_t reg_addr, uint8_t *read)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t val = 0;
  uint8_t rx_len = 0;

  while (rx_len != 1)
  {
    if (i2c_take_by_chipid(data->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      rx_len = xI2CMasterWriteRead(i2c_interf, i2c_addr, &reg_addr, 1, &val, 1);
      i2c_give(i2c_interf);
    }

    if (rx_len != 1)
    {
#ifdef DEBUG
      printf("UCD read error @ 0x%x\n", reg_addr);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid too much unnecessary I2C traffic */
    }
  }

  *read = val;
  return rx_len;
}

/**
 * @brief Reads a 16-bit word from a UCD90xxx device register.
 *
 * This function reads a 16-bit word from the specified register address of a UCD90xxx device.
 * It uses the I2C interface to communicate with the device. The function will retry the read
 * operation until it successfully reads two bytes of data. If the read operation fails, it will
 * delay for a short period to avoid excessive I2C traffic and then retry.
 *
 * The function combines the two bytes read from the device into a single 16-bit word, with the
 * first byte read being the most significant byte (MSB) and the second byte being the least
 * significant byte (LSB).
 *
 * @param data Pointer to the UCD90xxx data structure containing the chip ID and other relevant information.
 * @param reg_addr The register address from which to read the 16-bit word.
 * @param read Pointer to a variable where the read 16-bit word will be stored.
 *
 * @return The number of bytes successfully read (2 if successful).
 */
uint8_t ucd_read_word(ucd_data_t *data, uint8_t reg_addr, uint16_t *read)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t val[2] = {0};
  uint8_t rx_len = 0;

  /* Retry until two bytes are successfully read */
  while (rx_len != 2)
  {
    /* Acquire I2C interface and address using the chip ID */
    if (i2c_take_by_chipid(data->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      /* Perform the I2C read operation */
      rx_len = xI2CMasterWriteRead(i2c_interf, i2c_addr, &reg_addr, 1, val, 2);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    /* If the read operation fails, log the error and retry after a delay */
    if (rx_len != 2)
    {
#ifdef DEBUG
      printf("UCD read error @ 0x%x\n", reg_addr);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }

  /* Combine the two bytes into a 16-bit word */
  *read = (val[1] << 8) | (val[0]);

  return rx_len; /* Return the number of bytes read (2) */
}

/**
 * @brief Reads a block of data from a UCD90xxx device register.
 *
 * This function reads a block of data from the specified register address of a UCD90xxx device.
 * It uses the I2C interface to communicate with the device. The function will retry the read
 * operation until it successfully reads the specified number of bytes. If the read operation
 * fails, it will delay for a short period to avoid excessive I2C traffic and then retry.
 *
 * @param data Pointer to the UCD90xxx data structure containing the chip ID and other relevant information.
 * @param reg_addr The register address from which to read the block of data.
 * @param read Pointer to a buffer where the read data will be stored.
 * @param length The number of bytes to read.
 *
 * @return The number of bytes successfully read.
 */
uint8_t ucd_read_block(ucd_data_t *data, uint8_t reg_addr, uint8_t *read, uint8_t length)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t rx_len = 0;

  /* Retry until the specified number of bytes are successfully read */
  while (rx_len != length)
  {
    /* Acquire I2C interface and address using the chip ID */
    if (i2c_take_by_chipid(data->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      /* Perform the I2C read operation */
      rx_len = xI2CMasterWriteRead(i2c_interf, i2c_addr, &reg_addr, 1, read, length);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    /* If the read operation fails, log the error and retry after a delay */
    if (rx_len != length)
    {
#ifdef DEBUG
      printf("UCD read error @ 0x%x\n", reg_addr);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }

  return rx_len; /* Return the number of bytes read */
}

/**
 * @brief Writes a single byte to a UCD90xxx device register.
 *
 * This function writes a single byte to the specified register address of a UCD90xxx device.
 * It uses the I2C interface to communicate with the device. The function will retry the write
 * operation until it successfully writes one byte of data. If the write operation fails, it will
 * delay for a short period to avoid excessive I2C traffic and then retry.
 *
 * @param data Pointer to the UCD90xxx data structure containing the chip ID and other relevant information.
 * @param reg_addr The register address to which the byte will be written.
 * @param written Pointer to the byte value to be written.
 *
 * @return The number of bytes successfully written (1 if successful).
 */
uint8_t ucd_write_byte(ucd_data_t *data, uint8_t reg_addr, uint8_t *written)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t val[2] = {reg_addr, *written};
  uint8_t len = 0;

  /* Retry until two bytes are successfully written */
  while (len != 2)
  {
    /* Acquire I2C interface and address using the chip ID */
    if (i2c_take_by_chipid(data->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      /* Perform the I2C write operation */
      len = xI2CMasterWrite(i2c_interf, i2c_addr, val, 2);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    /* If the write operation fails, log the error and retry after a delay */
    if (len != 2)
    {
#ifdef DEBUG
      printf("UCD write error @ 0x%x\n", reg_addr);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }

  return 1; /* Return the number of bytes written (1) */
}

/**
 * @brief Writes a 16-bit word to a UCD90xxx device register.
 *
 * This function writes a 16-bit word to the specified register address of a UCD90xxx device.
 * It uses the I2C interface to communicate with the device. The function will retry the write
 * operation until it successfully writes three bytes of data (register address, MSB, and LSB).
 * If the write operation fails, it will delay for a short period to avoid excessive I2C traffic
 * and then retry.
 *
 * @param data Pointer to the UCD90xxx data structure containing the chip ID and other relevant information.
 * @param reg_addr The register address to which the 16-bit word will be written.
 * @param written Pointer to the 16-bit word value to be written.
 *
 * @return The number of bytes successfully written (2 if successful).
 */
uint8_t ucd_write_word(ucd_data_t *data, uint8_t reg_addr, uint16_t *written)
{
  uint8_t i2c_interf, i2c_addr;
  uint8_t val[3] = {reg_addr, (*written) & 0xFF, ((*written) >> 8) & 0xFF};
  uint8_t tx_len = 0;

  /* Retry until three bytes are successfully written */
  while (tx_len != 3)
  {
    /* Acquire I2C interface and address using the chip ID */
    if (i2c_take_by_chipid(data->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY) == pdTRUE)
    {
      /* Perform the I2C write operation */
      tx_len = xI2CMasterWrite(i2c_interf, i2c_addr, val, 3);
      i2c_give(i2c_interf); /* Release the I2C interface */
    }

    /* If the write operation fails, log the error and retry after a delay */
    if (tx_len != 3)
    {
#ifdef DEBUG
      printf("UCD write error @ 0x%x\n", reg_addr);
#endif
      vTaskDelay(pdMS_TO_TICKS(UCD_TIMEOUT)); /* Avoid excessive I2C traffic */
    }
  }

  return 2; /* Return the number of bytes written (2) */
}

/**
 * @brief Converts a PMBus value to a sensor data record (SDR) value.
 *
 * This function converts a PMBus value to a sensor data record (SDR) value using the
 * linearization formula specified in the SDR. The formula is:
 *   value = [(M * x + B * 10^(B_exp)) * 10^(R_exp)]
 * where:
 *   - M is the slope coefficient
 *   - B is the offset coefficient
 *   - B_exp is the offset exponent
 *   - R_exp is the result exponent
 *
 * @param data Pointer to the UCD90xxx data structure containing the sensor information.
 * @param value The PMBus value to be converted.
 *
 * @return The converted SDR value as a 16-bit unsigned integer.
 */
uint16_t pmbus2sdr_convert(ucd_data_t *data, float value)
{
  /* Extract the SDR coefficients and exponents */
  SDR_type_01h_t *sdr = (SDR_type_01h_t *)data->sensor->sdr;
  uint8_t M = sdr->M;
  uint8_t B = sdr->B;
  uint8_t Rexp_Bexp = sdr->Rexp_Bexp;

  /* Extract and convert the exponents */
  uint8_t uRexp = Rexp_Bexp >> 4;
  uint8_t uBexp = Rexp_Bexp & 0xF;
  int8_t Rexp = (uRexp > 0x7) ? uRexp - 0x10 : uRexp;
  int8_t Bexp = (uBexp > 0x7) ? uBexp - 0x10 : uBexp;

  /* Calculate the converted value using the linearization formula */
  float converted_value = (value / pow(10, Rexp) - B * pow(10, Bexp)) / M;

  /* Return the result as a 16-bit unsigned integer */
  return (uint16_t)converted_value;
}

/**
 * @brief Reads voltage data from a UCD90xxx device.
 *
 * This function reads the voltage data from the specified UCD90xxx device. It first sets the page
 * to the appropriate value based on the sensor's OEM field, then reads the voltage mode and the
 * actual voltage value from the device. The voltage value is then stored in the sensor's readout
 * value field.
 *
 * @param data Pointer to the UCD90xxx data structure containing the sensor and chip ID information.
 */
void ucd_read_voltages(ucd_data_t *data)
{
  SDR_type_01h_t *sdr = (SDR_type_01h_t *)data->sensor->sdr;
  uint8_t page = sdr->OEM;
  uint8_t mode;
  int8_t M_exp;
  uint16_t adc;

  /* Set the page to the appropriate value */
  ucd_write_byte(data, UCD_REG_PAGE, &page);

  /* Read the voltage mode */
  ucd_read_byte(data, UCD_REG_VOUT_MODE, &mode);
  /* mode is two's complement for a 5-bit value */
  M_exp = (mode > 0xF) ? (mode - 0x20) : mode;

  /* Read the voltage value */
  ucd_read_word(data, UCD_REG_READ_VOUT, &adc);

  /* Calculate the actual voltage value based on the mode */
  /* Nominal reading Voltage = adc * 2^(M_exp) = [(M * x + B * 10^(B_exp)) * 10^(R_exp)] */
  data->sensor->readout_value = pmbus2sdr_convert(data, adc * pow(2, M_exp));

  /* Optionally, print debug information */
  // #ifdef DEBUG
  //   printf("ucd read mode: 0x%x\n", mode);
  //   printf("ucd read vout: 0x%x\n", adc);
  //   printf("ucd calculated voltage: %d\n", data->sensor->readout_value);
  // #endif
}

void ucd_read_current(ucd_data_t *data)
{
  SDR_type_01h_t *sdr = (SDR_type_01h_t *)data->sensor->sdr;
  uint8_t page = sdr->OEM;
  uint8_t mode;
  int8_t M_exp;
  uint16_t adc;
  int16_t iadc;

  /* Set the page to the appropriate value */
  ucd_write_byte(data, UCD_REG_PAGE, &page);

  /* Read the current value */
  ucd_read_word(data, UCD_REG_READ_IOUT, &adc);

  mode = (adc >> 11) & 0x1F;
  M_exp = (mode > 0xF) ? (mode - 0x20) : mode;

  adc &= 0x7FF;
  iadc = (adc > 0x3FF) ? (adc - 0x800) : adc;

  /* Calculate the actual current value based on the mode */
  /* Nominal reading current = iadc * 2^(M_exp) = [(M * x + B * 10^(B_exp)) * 10^(R_exp)] */
  data->sensor->readout_value = pmbus2sdr_convert(data, iadc * pow(2, M_exp));
  ;

  /* Optionally, print debug information */
  // #ifdef DEBUG
  //   printf("ucd read Iout: 0x%x\n", adc);
  //   printf("ucd calculated current: %d\n", data->sensor->readout_value);
  // #endif
}

/**
 * @brief Reads temperature data from a UCD90xxx device.
 *
 * This function reads the temperature data from the specified UCD90xxx device. It reads the temperature
 * value from the device and converts it using the PMBus to SDR conversion formula. The temperature value
 * is then stored in the sensor's readout value field.
 *
 * @param data Pointer to the UCD90xxx data structure containing the sensor and chip ID information.
 */
void ucd_read_temperature(ucd_data_t *data)
{
  SDR_type_01h_t *sdr = (SDR_type_01h_t *)data->sensor->sdr;
  uint8_t page = sdr->OEM;
  uint8_t mode;
  int8_t M_exp;
  uint16_t adc;
  int16_t iadc;

  /* Set the page to the appropriate value */
  ucd_write_byte(data, UCD_REG_PAGE, &page);

  /* Read the temperature value */
  ucd_read_word(data, UCD_REG_READ_TEMPERATURE_1, &adc);

  /* Extract the mode and exponent from the read value */
  mode = (adc >> 11) & 0x1F;
  M_exp = (mode > 0xF) ? (mode - 0x20) : mode;

  /* Convert the ADC value to a signed integer */
  adc &= 0x7FF;
  iadc = (adc > 0x3FF) ? (adc - 0x800) : adc;

  /* Calculate the actual temperature value based on the mode */
  /* Nominal reading temperature = iadc * 2^(M_exp) = [(M * x + B * 10^(B_exp)) * 10^(R_exp)] */
  data->sensor->readout_value = pmbus2sdr_convert(data, iadc * pow(2, M_exp));

  /* Optionally, print debug information */
  // #ifdef DEBUG
  //   printf("ucd read temperature: 0x%x\n", adc);
  //   printf("ucd calculated temperature: %d\n", data->sensor->readout_value);
  // #endif
}

/**
 * @brief Initializes the UCD90xxx sensor data and creates the task for handling sensor updates.
 *
 * This function initializes the UCD90xxx sensor data by iterating through the SDR (Sensor Data Record) table
 * to find all UCD90xxx entries. It then assigns the appropriate chip ID and sensor pointer to the `ucd_data`
 * array. Finally, it creates a FreeRTOS task to handle periodic updates of the UCD90xxx sensors.
 */
void ucd_init(void)
{
  sensor_t *tmp_sensor;
  uint8_t ch_num;

  /* Create the task for handling UCD90xxx sensor updates */
  xTaskCreate(vTaskUCD, "UCD90XXX", 256, (void *)NULL, tskUCDSENSOR_PRIORITY, &vTaskUCD_Handle);

  /* Iterate through the SDR Table to find all the UCD90xxx entries */
  for (tmp_sensor = sdr_head; tmp_sensor != NULL; tmp_sensor = tmp_sensor->next)
  {
    /* Skip sensors that do not have a task handle assigned */
    if (tmp_sensor->task_handle == NULL)
    {
      continue;
    }

    /* Check if this task should update the selected SDR */
    if (*(tmp_sensor->task_handle) != vTaskUCD_Handle)
    {
      continue;
    }

    /* Assign the chip ID and sensor pointer to the ucd_data array */
    for (ch_num = 0; ch_num < SDR_CH_COUNT; ch_num++)
    {
      if (!(ucd_data[ch_num].chipid))
      {
        ucd_data[ch_num].chipid = tmp_sensor->chipid;
        ucd_data[ch_num].sensor = tmp_sensor;
        break;
      }
    }
  }
}
