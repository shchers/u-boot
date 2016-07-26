/*
 * Copyright (C) 2016 Sergey Shcherbakov <shchers@gmail.com>
 * As a base used mx23_olinuxino.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_MX23_SK_H__
#define __CONFIGS_MX23_SK_H__

/* System configurations */
#define CONFIG_MX23				/* i.MX23 SoC */
#define CONFIG_MACH_TYPE	4105

/* U-Boot Commands */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_LED
#define CONFIG_CMD_MMC
#define CONFIG_CMD_USB
#define CONFIG_CMD_BOOTZ

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x04000000	/* Max 64 MB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OVERWRITE

/* Environment is in MMC */
#if defined(CONFIG_CMD_MMC) && defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET	(256 * 1024)
#define CONFIG_ENV_SIZE		(16 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV	0
#endif

/* Status LED */
#define CONFIG_STATUS_LED
#define CONFIG_GPIO_LED
#define CONFIG_BOARD_SPECIFIC_LED
#define STATUS_LED_BOOT		0
#define STATUS_LED_BIT		MX23_PAD_SSP1_DETECT__GPIO_2_1
#define STATUS_LED_STATE	STATUS_LED_ON
#define STATUS_LED_PERIOD	(CONFIG_SYS_HZ / 2)

/* USB */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_MXS_PORT0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#define CONFIG_USB_STORAGE
#endif

/* Ethernet */
#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_SPI

/* KS8851 connected to SSP1 */
#define CONFIG_KS8851
#define CONFIG_KS8851_SPI_BUS       0
#define CONFIG_KS8851_SPI_CS        0
#define CONFIG_KS8851_SPI_CLOCK     24000000

/*
 * No EEPROM with MAC address connected to KS8851
 * So, MAC address will be generated.
 */
#define CONFIG_NET_RANDOM_ETHADDR
#define CONFIG_LIB_RAND
#endif

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_MXS_SPI
#define CONFIG_DEFAULT_SPI_BUS      0
#define CONFIG_DEFAULT_SPI_MODE     SPI_MODE_0
#endif

/* Booting Linux */
#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOTFILE		"zImage"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"update_sd_firmware_filename=u-boot.sd\0" \
	"update_sd_firmware="		/* Update the SD firmware partition */ \
		"if mmc rescan ; then "	\
		"if tftp ${update_sd_firmware_filename} ; then " \
		"setexpr fw_sz ${filesize} / 0x200 ; "	/* SD block size */ \
		"setexpr fw_sz ${fw_sz} + 1 ; "	\
		"mmc write ${loadaddr} 0x800 ${fw_sz} ; " \
		"fi ; "	\
		"fi\0" \
	"script=boot.scr\0"	\
	"zimage=boot/zImage\0" \
	"console=ttyAMA0\0" \
	"fdt_file=boot/imx23-sk.dtb\0" \
	"fdt_addr=0x41000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcroot=/dev/mmcblk0p2 rw rootwait\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript="  \
		"ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; "	\
		"source\0" \
	"loadzimage=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${zimage}\0" \
	"loadfdt=ext4load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"usb start; " \
		"run netargs; "	\
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${zimage}; " \
		"if test ${boot_fdt} = yes; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi;" \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadzimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_MX23_SK_H__ */
