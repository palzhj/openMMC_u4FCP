#include "port.h"
#include "pin_mapping.h"

const t_ssp_pin ssp_pins[MAX_SSP_INTERFACES] = {
    [FPGA_SPI] = {
        .port = 1,
        .sck_pin = 20,
        .sck_func = IOCON_FUNC3,
        .ssel_pin = 21,
        .ssel_func = IOCON_FUNC0,
        .miso_pin = 23,
        .miso_func = IOCON_FUNC3,
        .mosi_pin = 24,
        .mosi_func = IOCON_FUNC3,
    },
    [FLASH_SPI] = {
        .port = 0,
        .sck_pin = 7,
        .sck_func = IOCON_FUNC2,
        .ssel_pin = 6,
        .ssel_func = IOCON_FUNC0,
        .miso_pin = 8,
        .miso_func = IOCON_FUNC2,
        .mosi_pin = 9,
        .mosi_func = IOCON_FUNC2,
    },
    [DAC_VADJ_SPI] = {
        .port = 0,
        .sck_pin = 15,
        .sck_func = IOCON_FUNC2,
        .ssel_pin = 16,
        .ssel_func = IOCON_FUNC0,
        /* Leave the MISO pin as GPIO as it's used for another purpose on the board */
        .miso_pin = 17,
        .miso_func = IOCON_FUNC0,
        .mosi_pin = 18,
        .mosi_func = IOCON_FUNC2,
    }
};

static t_ssp_config ssp_cfg[MAX_SSP_INTERFACES] = {
    [FPGA_SPI] = {
        .lpc_id = LPC_SSP0,
        .irq = SSP0_IRQn,
        .pin_cfg = &ssp_pins[FPGA_SPI]
    },
    [FLASH_SPI] = {
        .lpc_id = LPC_SSP1,
        .irq = SSP1_IRQn,
        .pin_cfg = &ssp_pins[FLASH_SPI]
    },
    [DAC_VADJ_SPI] = {
        .lpc_id = LPC_SSP0,
        .irq = SSP0_IRQn,
        .pin_cfg = &ssp_pins[DAC_VADJ_SPI]
    },
};

/* Maybe use semaphores to control the access to SSP0 */
uint8_t active_SSP0 = 0xFF;

void SSP0_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    Chip_SSP_DATA_SETUP_T * xf_setup = &ssp_cfg[active_SSP0].xf_setup;

    /* Disable SSP interrupts */
    Chip_SSP_Int_Disable(LPC_SSP0);

    if (ssp_cfg[active_SSP0].frame_size <= 8) {
        Chip_SSP_Int_RWFrames8Bits(LPC_SSP0, xf_setup);
    }
    else {
        Chip_SSP_Int_RWFrames16Bits(LPC_SSP0, xf_setup);
    }

    if ((xf_setup->rx_cnt != xf_setup->length) || (xf_setup->tx_cnt != xf_setup->length)) {
	/* Enable all interrupts, we're going to read/write more data */
        Chip_SSP_Int_Enable(LPC_SSP0);
    }
    else {
        /* Transfer is completed, notify the caller task */
	vTaskNotifyGiveFromISR(ssp_cfg[active_SSP0].caller_task, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void SSP1_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    Chip_SSP_DATA_SETUP_T * xf_setup = &ssp_cfg[FLASH_SPI].xf_setup;

    /* Disable SSP interrupts */
    Chip_SSP_Int_Disable(LPC_SSP1);

    if (ssp_cfg[FLASH_SPI].frame_size <= 8) {
        Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, xf_setup);
    }
    else {
        Chip_SSP_Int_RWFrames16Bits(LPC_SSP1, xf_setup);
    }

    if ((xf_setup->rx_cnt != xf_setup->length) || (xf_setup->tx_cnt != xf_setup->length)) {
	/* Enable all interrupts, we're going to read/write more data */
        Chip_SSP_Int_Enable(LPC_SSP1);
    }
    else {
        /* Transfer is completed, notify the caller task */
	vTaskNotifyGiveFromISR(ssp_cfg[FLASH_SPI].caller_task, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void ssp_ssel_control( uint8_t id, t_ssel_state state )
{
    gpio_set_pin_state( ssp_pins[id].port, ssp_pins[id].ssel_pin, state );
}

void ssp_init( uint8_t id, uint32_t bitrate, uint8_t frame_sz, bool master_mode, bool poll )
{
    ssp_cfg[id].polling = poll;
    ssp_cfg[id].frame_size = frame_sz;
    ssp_cfg[id].master_mode = master_mode;
    ssp_cfg[id].bitrate = bitrate;

    /* Set up clock and muxing for SSP0/1 interface */
    /* Slave Select (SSEL/FCS_B) is left as GPIO so we can send more than one byte without this pin going high (default operation of SSP interface) */

    Chip_IOCON_Init(LPC_IOCON);

    Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[id].port, ssp_pins[id].sck_pin, IOCON_MODE_PULLDOWN, ssp_pins[id].sck_func);
    Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[id].port, ssp_pins[id].ssel_pin, IOCON_MODE_PULLUP, ssp_pins[id].ssel_func);
    Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[id].port, ssp_pins[id].mosi_pin, IOCON_MODE_INACT, ssp_pins[id].mosi_func);
    Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[id].port, ssp_pins[id].miso_pin, IOCON_MODE_INACT, ssp_pins[id].miso_func);

    if (ssp_pins[id].ssel_func == 0) {
	gpio_set_pin_dir( ssp_pins[id].port, ssp_pins[id].ssel_pin, OUTPUT);
	gpio_set_pin_state( ssp_pins[id].port, ssp_pins[id].ssel_pin, HIGH);
    }

#if 0
    /* Reset the other port to GPIO mode */
    if ( id == FPGA_SPI && active_SSP0 == DAC_VADJ_SPI) {
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[DAC_VADJ_SPI].port, ssp_pins[DAC_VADJ_SPI].sck_pin, IOCON_MODE_INACT, 0);
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[DAC_VADJ_SPI].port, ssp_pins[DAC_VADJ_SPI].mosi_pin, IOCON_MODE_INACT, 0);
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[DAC_VADJ_SPI].port, ssp_pins[DAC_VADJ_SPI].miso_pin, IOCON_MODE_INACT, 0);
    } else if (id == DAC_VADJ_SPI && active_SSP0 == FPGA_SPI) {
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[FPGA_SPI].port, ssp_pins[FPGA_SPI].sck_pin, IOCON_MODE_INACT, 0);
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[FPGA_SPI].port, ssp_pins[FPGA_SPI].mosi_pin, IOCON_MODE_INACT, 0);
	Chip_IOCON_PinMux(LPC_IOCON, ssp_pins[FPGA_SPI].port, ssp_pins[FPGA_SPI].miso_pin, IOCON_MODE_INACT, 0);
    }
#endif

    Chip_SSP_Init(ssp_cfg[id].lpc_id);
    Chip_SSP_SetBitRate(ssp_cfg[id].lpc_id, bitrate);
    Chip_SSP_SetMaster(ssp_cfg[id].lpc_id, master_mode);
    Chip_SSP_SetFormat(ssp_cfg[id].lpc_id, (frame_sz-1), SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
    Chip_SSP_Enable(ssp_cfg[id].lpc_id);

    if (!poll) {
        /* Configure interruption priority and enable it */
	NVIC_SetPriority( ssp_cfg[id].irq, configMAX_SYSCALL_INTERRUPT_PRIORITY );
        NVIC_EnableIRQ( ssp_cfg[id].irq );
    }

    if (ssp_cfg[id].lpc_id == LPC_SSP0) {
	active_SSP0 = id;
    }
}

void ssp_write_read( uint8_t id, uint8_t *tx_buf, uint8_t tx_len, uint8_t *rx_buf, uint8_t rx_len )
{
    Chip_SSP_DATA_SETUP_T * data_st = &ssp_cfg[id].xf_setup;

    ssp_cfg[id].caller_task = xTaskGetCurrentTaskHandle();
    data_st->tx_cnt = 0;
    data_st->rx_cnt = 0;
    data_st->tx_data = tx_buf;
    data_st->rx_data = rx_buf;
    data_st->length = rx_len+tx_len;

    if (ssp_cfg[id].polling) {
	Chip_SSP_RWFrames_Blocking(ssp_cfg[id].lpc_id, data_st);

    } else {
	Chip_SSP_Int_FlushData(ssp_cfg[id].lpc_id);

	/* Enable interrupt-based data transmission */
	Chip_SSP_Int_Enable( ssp_cfg[id].lpc_id );

	/* User defined timeout ? */
	/* Wait until the transfer is finished */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
