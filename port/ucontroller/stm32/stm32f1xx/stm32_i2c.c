/*
 *   openMMC  --
 *
 *   Copyright (C) 2015  Henrique Silva  <henrique.silva@lnls.br>
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
 */

/*!
 * @file stm32_i2c.c
 * @author Henrique Silva <henrique.silva@lnls.br>, LNLS
 * @date March 2016
 * @author Jie Zhang <zhj@ihep.ac.cn>, IHEP
 * @date Feb. 2025
 *
 * @brief I2C driver for STM32F10x
 */

#include "port.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#define SLAVE_MASK 0xFF

/* State machine handler for I2C1 and I2C2 */
static void i2c_state_handling(I2C_ID_T id)
{
  // if (Chip_I2C_IsMasterActive(id))
  // {
  //   Chip_I2C_MasterStateHandler(id);
  // }
  // else
  // {
  //   Chip_I2C_SlaveStateHandler(id);
  // }
}

void I2C1_IRQHandler(void)
{
  i2c_state_handling(I2C1_ID);
}

void I2C2_IRQHandler(void)
{
  i2c_state_handling(I2C2_ID);
}

void vI2CConfig(I2C_ID_T id, uint32_t speed)
{
  I2C_TypeDef *I2Cx;
  // IRQn_Type irq;
  GPIO_InitTypeDef GPIO_InitStruct;
  // NVIC_InitTypeDef NVIC_InitStruct;
  I2C_InitTypeDef I2C_InitStructure;

  /*!< I2C_SCL_GPIO_CLK and I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  switch (id)
  {
  case I2C2_ID:
    I2Cx = I2C2;
    // irq = I2C2_EV_IRQn;
    /*!< I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); // Enable I2C2 clock
    /*!< GPIO configuration */
    /*!< Configure I2C pins: SCL(PB10) and SDA(PB11) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    break;
  default:
    I2Cx = I2C1;
    // irq = I2C1_EV_IRQn;
    /*!< I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); // Enable I2C1 clock
    /*!< GPIO configuration */
    /*!< Configure I2C pins: SCL(PB6) and SDA(PB7) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  I2C_StructInit(&I2C_InitStructure); // Reset I2C_InitStructure to default values

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                // Set to I2C mode
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                        // Standard duty cycle (50%)
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;                                 // No address for master mode
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                               // Enable acknowledgment
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7-bit addressing mode
  I2C_InitStructure.I2C_ClockSpeed = speed;                                 // Set I2C clock speed to 100kHz (can be adjusted)

  I2C_Init(I2Cx, &I2C_InitStructure); // Initialize the I2C peripheral

  // Enable the I2C peripheral in master mode
  I2C_Cmd(I2Cx, ENABLE); // Enable I2C peripheral

  // Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
}

/**
 * @brief  Configures the I2C peripheral to operate as a slave device.
 *
 * This function configures the I2C peripheral in slave mode with the given address.
 * It enables the necessary interrupts and sets the slave address. The slave device
 * can then send and receive data from the master device.
 *
 * @param  id          I2C peripheral ID (either I2C1_ID or I2C2_ID)
 * @param  slave_addr  7-bit I2C slave address (the lower 7 bits of the address)
 */
void vI2CSlaveSetup(I2C_ID_T id, uint8_t slave_addr)
{
  I2C_TypeDef *I2Cx;
  switch (id)
  {
  case I2C2_ID:
    I2Cx = I2C2;
    break;

  default:
    I2Cx = I2C1;
  }
  // 3. Initialize the I2C peripheral for slave mode
  I2C_InitTypeDef I2C_InitStructure;
  I2C_StructInit(&I2C_InitStructure); // Reset I2C_InitStructure to default values

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                // Set to I2C mode
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                        // Standard duty cycle (50%)
  I2C_InitStructure.I2C_OwnAddress1 = slave_addr << 1;                      // Set the slave address (left-shifted for 8-bit address)
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                               // Enable acknowledgment
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // Set for 7-bit addressing

  I2C_Init(I2Cx, &I2C_InitStructure); // Initialize the I2C peripheral

  // 4. Enable the I2C peripheral in slave mode
  I2C_Cmd(I2Cx, ENABLE); // Enable I2C peripheral

  // 5. Enable the interrupts (optional based on application)
  // I2C_ITConfig(I2Cx, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, ENABLE);  // Enable buffer, event, and error interrupts

  // 6. Optionally, enable the address match interrupt if required
  // I2C_ITConfig(I2Cx, I2C_IT_ADDR, ENABLE);  // Enable the address interrupt to detect when the slave address is matched
}

static TaskHandle_t slave_task_id;
// I2C_XFER_T slave_cfg;
// I2C_XFER_T slave_dummy;
uint8_t recv_msg[i2cMAX_MSG_LENGTH];
uint8_t recv_msg_dummy[i2cMAX_MSG_LENGTH];
uint8_t recv_bytes;

uint8_t xI2CSlaveReceive(I2C_ID_T id, uint8_t *rx_buff, uint8_t buff_len, uint32_t timeout)
{
  uint8_t bytes_to_copy = 0;
  slave_task_id = xTaskGetCurrentTaskHandle();

  if (ulTaskNotifyTake(pdTRUE, timeout) == pdTRUE)
  {
    if (recv_bytes > buff_len)
    {
      bytes_to_copy = buff_len;
    }
    else
    {
      bytes_to_copy = recv_bytes;
    }
    /* Copy the rx buffer to the pointer given */
    memcpy(rx_buff, &recv_msg[0], bytes_to_copy);
    return bytes_to_copy;
  }
  else
  {
    return 0;
  }
}

// static void I2C_Slave_Event(I2C_ID_T id, I2C_EVENT_T event)
// {
//   static BaseType_t xHigherPriorityTaskWoken;
//   switch (event)
//   {
//   case I2C_EVENT_DONE:
//     recv_bytes = i2cMAX_MSG_LENGTH - slave_cfg.rxSz;
//     slave_cfg.rxSz = i2cMAX_MSG_LENGTH;
//     slave_cfg.rxBuff = &recv_msg[0];

//     vTaskNotifyGiveFromISR(slave_task_id, &xHigherPriorityTaskWoken);
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

//   case I2C_EVENT_SLAVE_RX:
//     break;
//   default:
//     break;
//   }
// }

/**
 * @brief  Perform a combined I2C master write followed by read operation.
 *
 * This function sends a buffer of data to an I2C slave device, followed by a read
 * operation to receive data from the slave device. It performs a write followed by a read
 * on the same I2C transaction. The function handles timeouts and returns specific error
 * codes in case of failure.
 *
 * @param  id          I2C peripheral ID (either I2C1_ID or I2C2_ID)
 * @param  addr        7-bit I2C slave address (the lower 7 bits of the address)
 * @param  tx_buff     Pointer to the buffer that contains the data to be written
 * @param  tx_len      Length of the data to be written (in bytes)
 * @param  rx_buff     Pointer to the buffer to store the received data
 * @param  rx_len      Length of the data to be read (in bytes)
 *
 * @retval >0          Number of bytes successfully received.
 * @retval 0           Error occurred during the operation (timeouts or other issues).
 */
uint8_t xI2CMasterWriteRead(I2C_ID_T id, uint8_t addr, const uint8_t *tx_buff, uint8_t tx_len, uint8_t *rx_buff, uint8_t rx_len)
{
  uint32_t timeout = I2C_TIMEOUT; // Set timeout duration
  uint8_t i = 0;
  I2C_TypeDef *I2Cx;
  addr = addr << 1; // Shift the address to match I2C 8-bit format (7-bit address)

  // Select I2C peripheral based on the provided ID
  switch (id)
  {
  case I2C2_ID:
    I2Cx = I2C2;
    break;

  default:
    I2Cx = I2C1;
  }

  // Wait until the I2C peripheral is not busy
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Generate the START condition
  I2C_GenerateSTART(I2Cx, ENABLE);
  // Wait for the START condition to be transmitted
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Send the slave address with the write mode (LSB = 0 for write)
  I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter);
  // Wait for the slave to acknowledge the address
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }
  // Check if NACK was received after the address was sent
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
  {
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF); // Clear the NACK flag
    I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
    goto stop; // Timeout waiting for I2C to be free
  }

  // Send the data from the tx_buff
  for (i = 0; i < tx_len; i++)
  {
    I2C_SendData(I2Cx, tx_buff[i]);

    // Wait for the byte to be transmitted
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if (timeout-- == 0)
      {
        I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
        goto stop; // Timeout waiting for I2C to be free
      }
    }
  }

  // Generate a repeated START condition for the read operation
  I2C_GenerateSTART(I2Cx, ENABLE);

  // Wait for the repeated START condition to be transmitted
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Send the slave address with the read mode (LSB = 1 for read)
  I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);
  // Wait for the slave to acknowledge the address
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }
  // Check if NACK was received after the address was sent
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
  {
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF); // Clear the NACK flag
    goto stop; // Timeout waiting for I2C to be free
  }

  // Read data byte-by-byte
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  for (i = 0; i < rx_len; i++)
  {
    // If it's the last byte, disable acknowledgment and generate the STOP condition after receiving
    if (i == (rx_len - 1))
    {
      I2C_AcknowledgeConfig(I2Cx, DISABLE); // Disable Acknowledge after receiving the last byte
      I2C_GenerateSTOP(I2Cx, ENABLE);       // Generate STOP condition
    }

    // Wait for the data byte to be received
    timeout = I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE))
    {
      if (timeout-- == 0)
      {
        I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
        goto stop; // Timeout waiting for I2C to be free
      }
    }

    // Read the received data
    rx_buff[i] = I2C_ReceiveData(I2Cx);
  }

  stop:
    // Wait for the STOP condition to be generated
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF))
    {
      if (timeout-- == 0) return i;  // Timeout error during STOP condition, return the number of bytes received
    }

    // Return the number of bytes successfully received
    return i;
}

/**
 * @brief  Sends data to an I2C slave device in master mode.
 *
 * This function writes a buffer of data to an I2C slave device. It generates the
 * START condition, sends the slave address with the write flag, transmits the
 * data byte-by-byte, and then generates the STOP condition to complete the transmission.
 * The function handles timeouts and returns specific error codes in case of failure.
 *
 * @param  id          I2C peripheral ID (either I2C1_ID or I2C2_ID)
 * @param  slave_addr  7-bit I2C slave address (the lower 7 bits of the address)
 * @param  tx_buff     Pointer to the data buffer that will be sent to the slave
 * @param  buff_len    Length of the data buffer (number of bytes to send)
 *
 * @retval 0           Error occurred (timeout, NACK, or other failure).
 * @retval >0          Number of bytes successfully transmitted.
 */
uint8_t xI2CMasterWrite(I2C_ID_T id, uint8_t slave_addr, uint8_t *tx_buff, uint8_t buff_len)
{
  uint32_t timeout = I2C_TIMEOUT;  // Set timeout duration
  uint8_t i = 0;
  I2C_TypeDef *I2Cx;
  slave_addr = slave_addr << 1;  // Shift the address for I2C 8-bit address format

  // Select I2C peripheral based on provided ID
  switch (id)
  {
  case I2C2_ID:
    I2Cx = I2C2;
    break;

  default:
    I2Cx = I2C1;
  }

  // Wait until the I2C peripheral is not busy
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if (timeout-- == 0) goto stop; // Timeout waiting for I2C to be free
  }

  // Generate the START condition
  I2C_GenerateSTART(I2Cx, ENABLE);

  // Wait for the START condition to be transmitted
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (timeout-- == 0) goto stop; // Timeout on START condition
  }

  // Send the slave address with the write mode (LSB = 0 for write)
  I2C_Send7bitAddress(I2Cx, slave_addr, I2C_Direction_Transmitter);

  // Wait for the slave to acknowledge the address
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (timeout-- == 0) goto stop; // Timeout on address acknowledge
  }
  // Check if NACK was received after the address was sent
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
  {
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF); // Clear the NACK flag
    goto stop; // Timeout waiting for I2C to be free
  }

  // Send each byte of the data buffer
  for (i = 0; i < buff_len; i++)
  {
    I2C_SendData(I2Cx, tx_buff[i]);

    // Wait for the byte to be transmitted
    timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if (timeout-- == 0) goto stop; // Timeout on byte transmission
    }
    // Check for NACK after transmitting each byte
    if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
    {
      I2C_ClearFlag(I2Cx, I2C_FLAG_AF);  // Clear NACK flag
      goto stop;  // Stop if NACK is received
    }
  }

stop:
  // Generate the STOP condition
  I2C_GenerateSTOP(I2Cx, ENABLE);

  // Wait for the STOP condition to be generated
  timeout = I2C_TIMEOUT;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF))
  {
    if (timeout-- == 0) return i;  // Timeout during STOP condition, return the number of bytes sent
  }

  // Return the number of bytes successfully transmitted
  return i;
}

/**
 * @brief  Reads data from an I2C slave device in master mode.
 *
 * This function initiates a read operation from an I2C slave device. It generates the
 * START condition, sends the slave address with the read flag, and then reads the data
 * byte-by-byte into the provided buffer. The function handles timeouts and returns
 * specific error codes in case of failure.
 *
 * @param  id          I2C peripheral ID (either I2C1_ID or I2C2_ID)
 * @param  slave_addr  7-bit I2C slave address (the lower 7 bits of the address)
 * @param  rx_buff     Pointer to the buffer where the received data will be stored
 * @param  buff_len    Length of the buffer (number of bytes to read)
 *
 * @retval 0           Error occurred (timeout, NACK, or other failure).
 * @retval >0          Number of bytes successfully received.
 */
uint8_t xI2CMasterRead(I2C_ID_T id, uint8_t slave_addr, uint8_t *rx_buff, uint8_t buff_len)
{
  uint32_t timeout = I2C_TIMEOUT;  // Set timeout duration
  uint8_t i = 0;
  I2C_TypeDef *I2Cx;
  slave_addr = slave_addr << 1;  // Shift the address for I2C 8-bit address format

  // Select I2C peripheral based on provided ID
  switch (id)
  {
  case I2C2_ID:
    I2Cx = I2C2;
    break;

  default:
    I2Cx = I2C1;
  }

  // Wait until the I2C peripheral is not busy
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Generate the START condition
  I2C_GenerateSTART(I2Cx, ENABLE);

  // Wait for the START condition to be transmitted
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Send the slave address with the read mode (LSB = 1 for read)
  I2C_Send7bitAddress(I2Cx, slave_addr, I2C_Direction_Receiver);

  // Wait for the slave to acknowledge the address (check for NACK)
  timeout = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (timeout-- == 0)
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
    // Check if NACK was received after the address was sent
    if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
    {
      I2C_ClearFlag(I2Cx, I2C_FLAG_AF); // Clear the NACK flag
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
      goto stop; // Timeout waiting for I2C to be free
    }
  }

  // Read data byte-by-byte
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  for (i = 0; i < buff_len; i++)
  {
    // If it's the last byte, disable ACK and generate STOP condition after receiving
    if (i == (buff_len - 1))
    {
      I2C_AcknowledgeConfig(I2Cx, DISABLE);  // Disable Acknowledge after receiving last byte
      I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
    }

    // Wait for the data byte to be received
    timeout = I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE))
    {
      if (timeout-- == 0)
      {
        I2C_GenerateSTOP(I2Cx, ENABLE);        // Generate STOP condition
        goto stop; // Timeout waiting for I2C to be free
      }
    }

    // Read the received data
    rx_buff[i] = I2C_ReceiveData(I2Cx);
  }

  stop:
    // Wait for the STOP condition to be generated
    timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF))
    {
      if (timeout-- == 0) return i;  // Timeout error during STOP condition, return the number of bytes received
    }

    // Return the number of bytes successfully received
    return i;
}