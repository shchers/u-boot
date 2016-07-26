/*
 * Starterkit SK-i.MX233 board
 *
 * Copyright (C) 2016 Sergey Shcherbakov <shchers@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#ifdef CONFIG_CMD_SPI
#include <spi.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/iomux-mx23.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_STATUS_LED
#include <status_led.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define	MUX_CONFIG_SSP1	(MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP)
#define	MUX_CONFIG_SSP2	(MXS_PAD_3V3 | MXS_PAD_12MA | MXS_PAD_PULLUP)

#ifndef CONFIG_SPL_BUILD
const iomux_cfg_t iomux_setup[] = {
	/* DUART */
	MX23_PAD_PWM0__DUART_RX,
	MX23_PAD_PWM1__DUART_TX,

	/* SSP1-SPI (ks8851) */
	MX23_PAD_SSP1_SCK__SSP1_SCK | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_CMD__SSP1_CMD | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DATA0__SSP1_DATA0 | MUX_CONFIG_SSP1,
	MX23_PAD_SSP1_DATA3__SSP1_DATA3 |
		(MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_PULLUP),

	/* SSP2-MMC */
	MX23_PAD_GPMI_D00__SSP2_DATA0 | MUX_CONFIG_SSP2,
	MX23_PAD_GPMI_D01__SSP2_DATA1 | MUX_CONFIG_SSP2,
	MX23_PAD_GPMI_D02__SSP2_DATA2 | MUX_CONFIG_SSP2,
	MX23_PAD_GPMI_D03__SSP2_DATA3 | MUX_CONFIG_SSP2,
	MX23_PAD_GPMI_RDY1__SSP2_CMD | MUX_CONFIG_SSP2,
	MX23_PAD_GPMI_RDY0__SSP2_DETECT |
		(MXS_PAD_3V3 | MXS_PAD_12MA | MXS_PAD_NOPULL),
	MX23_PAD_GPMI_WRN__SSP2_SCK |
		(MXS_PAD_3V3 | MXS_PAD_12MA | MXS_PAD_NOPULL),
};
#endif

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);

#ifndef CONFIG_SPL_BUILD
	mxs_iomux_setup_multiple_pads(iomux_setup, ARRAY_SIZE(iomux_setup));
#endif

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

#ifdef	CONFIG_CMD_MMC
static int mmc_wp(int id)
{
	// Basic W/A, because "Write Protect" pin not connected to CPU
	return 0;
}

int board_mmc_init(bd_t *bis)
{
	// SD/MMC socket connected to SSP2. "1" meaning SSP2:)
	return mxsmmc_initialize(bis, 1, mmc_wp, NULL);
}
#endif

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

#if defined(CONFIG_STATUS_LED) && defined(STATUS_LED_BOOT)
	status_led_set(STATUS_LED_BOOT, STATUS_LED_STATE);
#endif

	return 0;
}

#ifdef	CONFIG_CMD_NET
int board_eth_init(bd_t *bis) {
#ifdef CONFIG_KS8851
	return ks8851_initialize(CONFIG_KS8851_SPI_BUS,
	                         CONFIG_KS8851_SPI_CS,
                             CONFIG_KS8851_SPI_CLOCK,
                             SPI_MODE_0);
#else
	return -1;
#endif
}
#endif
