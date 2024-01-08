/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
// #include <nuttx/i2c/i2c_master.h>
// #include <nuttx/sdio.h>
// #include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
// #include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#include <px4_arch/io_timer.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	// Setting the kill switch control gpio

	stm32_gpiowrite(GPIO_KILL_SW_EN, 0);
	stm32_gpiowrite(GPIO_KILL_SW1_NEG, 0);
	stm32_gpiowrite(GPIO_KILL_SW1_POS, 0);
	stm32_gpiowrite(GPIO_KILL_SW2_NEG, 0);
	stm32_gpiowrite(GPIO_KILL_SW2_POS, 0);

	// setting watchdog and reset GPIOs
	stm32_gpiowrite(GPIO_GBL_RST, 0);
	// stm32_gpiowrite(GPIO_WD_WDI, 0); // setting initial watchdog pin status

	// setting power contorl pins
	stm32_gpiowrite(GPIO_DCDC_IMU_EN, 1);	// enable IMU regulator
	stm32_gpiowrite(GPIO_DCDC_MSN, 1); 	// enable MSN regulator
	stm32_gpiowrite(GPIO_MSN_3V3_EN, 1);	//Disable MSN power (ssoc)
	stm32_gpiowrite(GPIO_MSN_5V_EN, 1);	// enable IMU power
	stm32_gpiowrite(GPIO_BURNER_EN, 0);	// Disable Burner enable
	stm32_gpiowrite(GPIO_UNREG_EN, 0);	// Disable UNREG power line

	// setting MSN control Pin
	stm32_gpiowrite(GPIO_MSN1_EN, 1);	//Disable MSN activation

	// setting flash control pins
	stm32_gpiowrite(GPIO_MFM_RST, 1);	// disabling flash reset
	stm32_gpiowrite(GPIO_MFM_HOLD, 1);	// disabling flash hold
	stm32_gpiowrite(GPIO_MFM_WP, 1);	// disabling hadware WP of flash

	stm32_gpiowrite(GPIO_SFM_RST, 1);	// disabling flash reset
	stm32_gpiowrite(GPIO_SFM_HOLD, 1);	// disabling flash hold
	stm32_gpiowrite(GPIO_SFM_WP, 1);	// disabling hadware WP of flash
	stm32_gpiowrite(GPIO_SFM_MODE, 0);
	stm32_gpiowrite(GPIO_MUX_EN, 0);
	// bool last = stm32_gpioread(GPIO_SPEKTRUM_PWR_EN);
	// // Keep Spektum on to discharge rail.
	// stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, 1);

	// Wait for the peripheral rail to reach GND.
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	// Re-enable power.
	// Switch the peripheral rail back on.
	// stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, last);

}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	// Configure the GPIO pins to outputs and keep them low.
	// for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
	// 	px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	// }

	/*
	 * On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 */

	if (status >= 0) {
		up_mdelay(400);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	// Reset all PWM to Low outputs.
	board_on_reset(-1);

	// Configure ADC pins.
	// stm32_configgpio(GPIO_ADC1_IN2);	/* BATT_VOLTAGE_SENS */
	// stm32_configgpio(GPIO_ADC1_IN3);	/* BATT_CURRENT_SENS */
	// stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	// stm32_configgpio(GPIO_ADC1_IN11);	/* RSSI analog in */


	// Configure Watchdog and Reset control pins.
	// stm32_configgpio(GPIO_WD_WDI);
	stm32_configgpio(GPIO_GBL_RST);

	// Configure KILL Switch Control and Monitoring GPIOs
	stm32_configgpio(GPIO_KILL_SW_EN);
	stm32_configgpio(GPIO_KILL_SW1_STAT); //input
	stm32_configgpio(GPIO_KILL_SW1_NEG);
	stm32_configgpio(GPIO_KILL_SW1_POS);
	stm32_configgpio(GPIO_KILL_SW2_STAT); //input
	stm32_configgpio(GPIO_KILL_SW2_NEG);
	stm32_configgpio(GPIO_KILL_SW2_POS);

	// Configure power supply control pins
	stm32_configgpio(GPIO_DCDC_IMU_EN);
	stm32_configgpio(GPIO_DCDC_MSN);
	stm32_configgpio(GPIO_MSN_3V3_EN);
	stm32_configgpio(GPIO_MSN_5V_EN);
	stm32_configgpio(GPIO_BURNER_EN);
	stm32_configgpio(GPIO_UNREG_EN);

	// Configure MSN Control Pins
	stm32_configgpio(GPIO_MSN1_EN);

	// Configure Flash control pins
	stm32_configgpio(GPIO_MFM_RST);
	stm32_configgpio(GPIO_MFM_HOLD);
	stm32_configgpio(GPIO_MFM_WP);
	stm32_configgpio(GPIO_SFM_RST);
	stm32_configgpio(GPIO_SFM_HOLD);
	stm32_configgpio(GPIO_SFM_WP);
	stm32_configgpio(GPIO_SFM_MODE);
	stm32_configgpio(GPIO_MUX_EN);
	// Configure SPI all interfaces GPIO & enable power.
	stm32_spiinitialize();
	// board_peripheral_reset(10);
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;
static struct spi_dev_s *spi4;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();
	board_peripheral_reset(10);

	// Configure the DMA allocator.
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

#if defined(SERIAL_HAVE_RXDMA)
	// set up the serial DMA polling at 1ms intervals for received bytes that have not triggered a DMA event.
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif
	if (board_hardfault_init(2, true) != 0) {
	}

	// Configure SPI-based devices.
	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
	} else {
		syslog(LOG_INFO, "[boot] Initialized SPI port 1\n");
		// Default SPI1 to 1MHz, IMUs
		SPI_SETFREQUENCY(spi1, 10000000);
		SPI_SETBITS(spi1, 8);
		SPI_SETMODE(spi1, SPIDEV_MODE0);
		up_udelay(20);
	}
	// Get the SPI port for the Ext ADC.
	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 2\n");
	} else {
		syslog(LOG_INFO, "[boot] Initialized SPI port 2\n");
		// Default SPI2 to 1MHz, ADC
		SPI_SETFREQUENCY(spi2, 10000000);
		SPI_SETBITS(spi2, 8);
		SPI_SETMODE(spi2, SPIDEV_MODE0);
		up_udelay(20);
	}


	// Get the SI port of Main FM
	spi3 = stm32_spibus_initialize(3);

	if(!spi3) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 3\n");
	} else  {
		syslog(LOG_INFO, "[boot] Initialized SPI port 3\n");
		/**
		 * Default SPI3 to 12MHz and de-assert the known chip selects.
		 * MT25QL has max SPI clock speed of 133MHz.
		 */
		// XXX start with 10.4 MHz and go up to 20 once validated.
		SPI_SETFREQUENCY(spi3, 20 * 1000 * 1000);
		SPI_SETBITS(spi3, 8);
		SPI_SETMODE(spi3, SPIDEV_MODE0);
		up_udelay(20);
	}

	spi4 = stm32_spibus_initialize(4);
	if (!spi4) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 4\n");

	} else {

		syslog(LOG_INFO, "[boot] Initialized SPI port 4\n");
		// Default SPI4 to 20 MHz
		SPI_SETFREQUENCY(spi4, 20 * 1000 * 1000);
		SPI_SETBITS(spi4, 8);
		SPI_SETMODE(spi4, SPIDEV_MODE0);
	}

	/* Configure the HW based on the manifest */

	px4_platform_configure();

	return OK;
}