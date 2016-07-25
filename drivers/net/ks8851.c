/*
 * 2012 Alexander Kudjashev <Kudjashev@gmail.com>
 *
 * Adapted ks8851_snl linux driver
 * 
 * Copyright 2009 Simtec Electronics
 *	http://www.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <common.h>
#include <net.h>
#include <spi.h>
#include <malloc.h>
#include <netdev.h>
#include <miiphy.h>
#include <linux/types.h>
#include "ks8851.h"

#define ETH_ALEN        6

/* shift for byte-enable data */
#define BYTE_EN(_x) ((_x) << 2)

/* turn register number and byte-enable mask into data for start of packet */
#define MK_OP(_byteen, _reg) (BYTE_EN(_byteen) | (_reg) << (8+2) | (_reg) >> 6)

union ks8851_tx_hdr {
    uchar   txb[6];
    ushort  txw[3];
};

typedef struct ks_device {
    struct eth_device   *dev;   /* back pointer */
    struct spi_slave    *slave;
    ushort fid;
    union  ks8851_tx_hdr txh;
    uchar  buff[2048];
} ks_dev_t;

/*
 * write a 8 bit register to ks8851 chip
 */
static void ks_reg8_write(ks_dev_t *ks, ushort reg, ushort val)
{
    ushort  tx[2];
    ushort  bit;

    bit = 1 << (reg & 3);

    tx[0] = MK_OP(bit, reg) | KS_SPIOP_WR;
    tx[1] = val;

    spi_xfer(ks->slave, 3 * 8, tx, NULL,
        SPI_XFER_BEGIN | SPI_XFER_END);
}

/*
 * write a 16 bit register to ks8851 chip
*/
static void ks_reg16_write(ks_dev_t *ks, ushort reg, ushort val)
{
    ushort tx[2];

    tx[0] = MK_OP(reg & 2 ? 0xC : 0x03, reg) | KS_SPIOP_WR;
    tx[1] = val;

    spi_xfer(ks->slave, 4 * 8, tx, NULL,
        SPI_XFER_BEGIN | SPI_XFER_END);
}

/*
 * issue read register command and return the data
 */
static void ks_reg_read(ks_dev_t *ks, ushort op, uchar *rx, ushort len)
{
    ushort tx = op | KS_SPIOP_RD;

    spi_xfer(ks->slave, 2 * 8, &tx, NULL, SPI_XFER_BEGIN);
    spi_xfer(ks->slave, len * 8, NULL, rx, SPI_XFER_END);
}

/*
 * read 8 bit register from device
 */
static uchar ks_reg8_read(ks_dev_t *ks, ushort reg)
{
    uchar rx = 0;

    ks_reg_read(ks, MK_OP(1 << (reg & 3), reg), &rx, 1);

    return rx;
}

/*
 * read 16 bit register from device
 */
static ushort ks_reg16_read(ks_dev_t *ks, ushort reg)
{
    ushort rx = 0;

    ks_reg_read(ks, MK_OP(reg & 2 ? 0xC : 0x3, reg), (uchar *)&rx, 2);

    return rx;
}

/*
 * read 32 bit register from device
 */
static uint ks_reg32_read(ks_dev_t *ks, ushort reg)
{
    uint rx = 0;

    ks_reg_read(ks, MK_OP(0x0f, reg), (uchar *)&rx, 4);

    return rx;
}

/*
 * read data from the receive fifo
 */
static void ks_fifo_read(ks_dev_t *ks, ushort len)
{
    uchar tx = KS_SPIOP_RXFIFO;

    spi_xfer(ks->slave, 1 * 8, &tx, NULL, SPI_XFER_BEGIN);
    spi_xfer(ks->slave, len * 8, NULL, ks->buff, SPI_XFER_END);
}

/*
 * write packet to TX FIFO
 */
static int ks_send(struct eth_device *dev,
                   volatile void *packet,
                   int length)
{
    ks_dev_t *ks = dev->priv;
    ushort fid = ks->fid++;

    spi_claim_bus(ks->slave);

    fid &= TXFR_TXFID_MASK;

    /* start header at txb[1] to align txw entries */
    ks->txh.txb[1] = KS_SPIOP_TXFIFO;
    ks->txh.txw[1] = fid;
    ks->txh.txw[2] = length;

    while (length > ks_reg16_read(ks, KS_TXMIR))
        udelay(10);

    ks_reg16_write(ks, KS_RXQCR, RXQCR_SDA | RXQCR_RXFCTE);

    spi_xfer(ks->slave, 5 * 8, &ks->txh.txb[1], NULL, SPI_XFER_BEGIN);
    spi_xfer(ks->slave, ALIGN(length, 4) * 8, (uchar *)packet, NULL, SPI_XFER_END);

    ks_reg16_write(ks, KS_RXQCR, RXQCR_RXFCTE);

    spi_claim_bus(ks->slave);

    return 0;
}

/*
 * receive packets from the host
 */
static int ks_recv(struct eth_device *dev)
{
    ks_dev_t *ks = dev->priv;
    uint    rxh;
    ushort  rxfc, rxlen;

    spi_claim_bus(ks->slave);

    if (ks_reg16_read(ks, KS_ISR) & IRQ_RXI) {
		ks_reg16_write(ks, KS_ISR, IRQ_RXI);
		rxfc = ks_reg8_read(ks, KS_RXFC);

		while (rxfc--) {
			rxh = ks_reg32_read(ks, KS_RXFHSR);
			/* the length of the frame includes the 32bit CRC */
			rxlen = rxh >> 16;

			/* setup Receive Frame Data Pointer Auto-Increment */
			ks_reg16_write(ks, KS_RXFDPR, RXFDPR_RXFPAI);

			/* start the packet dma process, and set auto-dequeue rx */
			ks_reg16_write(ks, KS_RXQCR, RXQCR_SDA | RXQCR_ADRFE | RXQCR_RXFCTE);

			if (rxlen > 4) {
				/* align the packet length to 4 bytes, and add 4 bytes
				   for dummy data */
				ks_fifo_read(ks, ALIGN(rxlen, 4) + 4);

				spi_release_bus(ks->slave);
				
				/* discard dummy data and frame header, 
				   packet length without frame header */		
				net_process_received_packet(ks->buff + 8, rxlen - 4);

				spi_claim_bus(ks->slave);
			}
   
			ks_reg16_write(ks, KS_RXQCR, RXQCR_RXFCTE);
		}
	}

    spi_release_bus(ks->slave);
    
    return 0;
}

/*
 * set ks8851 MAC address
*/
static int ks_write_hwaddr(struct eth_device *dev)
{
    ks_dev_t *ks = dev->priv;
    uchar *enetaddr = ks->dev->enetaddr;
    int i;

    spi_claim_bus(ks->slave);

    for (i = 0; i < ETH_ALEN; i++)
        ks_reg8_write(ks, KS_MAR(i), enetaddr[i]);

    spi_release_bus(ks->slave);
    
    return 0;
}

/*
 * set power mode of the device
 */
static void ks_powermode(ks_dev_t *ks, ushort pwrmode)
{
    ushort pmecr;

    pmecr = ks_reg16_read(ks, KS_PMECR);
    pmecr &= ~PMECR_PM_MASK;
    pmecr |= pwrmode;

    ks_reg16_write(ks, KS_PMECR, pmecr);
}

/*
 * configure network device
 */
static void ks_config(ks_dev_t *ks)
{
    /* auto-increment tx data, reset tx pointer */
    ks_reg16_write(ks, KS_TXFDPR, TXFDPR_TXFPAI);

    /* Enable QMU TxQ Auto-Enqueue frame */
    ks_reg16_write(ks, KS_TXQCR, TXQCR_AETFE);

    /* setup transmission parameters */
    ks_reg16_write(ks, KS_TXCR, TXCR_TXE | /* enable transmit process */
                                TXCR_TXPE |  /* pad to min length */
                                TXCR_TXCRC | /* add CRC */
                                TXCR_TXFCE); /* enable flow control */

    /* Setup Receive Frame Threshold 1 frame */
    ks_reg16_write(ks, KS_RXFCTR, 1 << RXFCTR_RXFCT_SHIFT);
	ks_reg16_write(ks, KS_RXQCR, RXQCR_RXFCTE);

    /* setup receiver control */
    ks_reg16_write(ks, KS_RXCR1, RXCR1_RXPAFMA | /* mac filter */
                                 RXCR1_RXFCE | /* enable flow control */
                                 RXCR1_RXUE |  /* unicast enable */
                                 RXCR1_RXBE |  /* broadcast enable */
                                 RXCR1_RXE);   /* enable rx block */

    /* transfer entire frames out in one go */
    ks_reg16_write(ks, KS_RXCR2, RXCR2_SRDBL_FRAME);
}

/*
 * ks8851 chip initialization
*/
static int ks_init(struct eth_device *dev, bd_t *bis)
{
    ks_dev_t *ks = dev->priv;
    ushort chip_id = 0;

    spi_claim_bus(ks->slave);

    /* issue a global soft reset to reset the device. */
    ks_reg16_write(ks, KS_GRR, GRR_GSR);
    udelay(1000); /* wait a short time to effect reset */
    ks_reg16_write(ks, KS_GRR, 0);
    udelay(1000); /* wait for condition to clear */
        
    /* simple check for a valid chip being connected to the bus */
    chip_id = ks_reg16_read(ks, KS_CIDER);
    printf("ks8851 chip ID=0x%x\n", chip_id);
    if ((chip_id & ~CIDER_REV_MASK) != CIDER_ID) {
        printf("Error: the ks8851 chip ID is wrong\n");
        return -1;
    }

    ks_config(ks);
    /* enable RX irq */
    ks_reg16_write(ks, KS_ISR, 0xffff);
    ks_reg16_write(ks, KS_IER, IRQ_RXI);

    spi_release_bus(ks->slave);
    
    return 0;
}

/*
 * close network device
 */
static void ks_halt(struct eth_device *dev)
{
    ks_dev_t *ks = dev->priv;

    spi_claim_bus(ks->slave);

    /* shutdown RX process */
    ks_reg16_write(ks, KS_RXCR1, 0x0000);

    /* shutdown TX process */
    ks_reg16_write(ks, KS_TXCR, 0x0000);

    /* set powermode to soft power down to save power */
    ks_powermode(ks, PMECR_PM_SOFTDOWN);

    spi_release_bus(ks->slave);
}

/*
 * ks_phy_reg - convert MII register into a ks8851 register
 */
static int ks_phy_reg(int reg)
{
    switch (reg) {
    case MII_BMCR:
        return KS_P1MBCR;
    case MII_BMSR:
        return KS_P1MBSR;
    case MII_PHYSID1:
        return KS_PHY1ILR;
    case MII_PHYSID2:
        return KS_PHY1IHR;
    case MII_ADVERTISE:
        return KS_P1ANAR;
    case MII_LPA:
        return KS_P1ANLPR;
    }

    return 0;
}

/*
 * ks_phy_read - MII interface PHY register read.
 */
static int ks_phy_read(const char *devname, 
                       uchar addr, 
                       uchar reg, 
                       ushort *val)
{
    struct eth_device *dev = eth_get_dev_by_name(devname);
    ks_dev_t *ks = dev->priv;
    ushort ksreg;

    ksreg = ks_phy_reg(reg);
    if (!ksreg)
        return -1;

    spi_claim_bus(ks->slave);
    *val = ks_reg16_read(ks, ksreg);
    spi_release_bus(ks->slave);
    
    return 0;
}

/*
 * ks_phy_write - MII interface PHY register write.
 */
static int ks_phy_write(const char *devname,
                         uchar addr, 
                         uchar reg, 
                         ushort val)
{
    struct eth_device *dev = eth_get_dev_by_name(devname);
    ks_dev_t *ks = dev->priv;
    ushort ksreg;

    ksreg = ks_phy_reg(reg);
    if (!ksreg)
        return -1;

    spi_claim_bus(ks->slave);
    ks_reg16_write(ks, ksreg, val);
    spi_release_bus(ks->slave);

	return 0;
}

/*
 * This is the only exported function.
 *
 * It may be called several times with different bus:cs combinations.
 */
int ks8851_initialize(uint bus, uint cs, uint max_hz, uint mode)
{
    struct eth_device *dev;
    ks_dev_t *ks;

    dev = malloc(sizeof(*dev));
    if (!dev) {
        return -1;
    }
    memset(dev, 0, sizeof(*dev));

    ks = malloc(sizeof(*ks));
    if (!ks) {
        free(dev);
        return -1;
    }
    memset(ks, 0, sizeof(*ks));

    /* try to setup the SPI slave */
    ks->slave = spi_setup_slave(bus, cs, max_hz, mode);
    if (!ks->slave) {
        printf("ks8851: invalid SPI device %i:%i\n", bus, cs);
        free(ks);
        free(dev);
        return -1;
    }

    ks->dev = dev;
    /* now fill the eth_device object */
    dev->priv = ks;
    dev->init = ks_init;
    dev->halt = ks_halt;
    dev->send = ks_send;
    dev->recv = ks_recv;
    dev->write_hwaddr = ks_write_hwaddr;
    sprintf(dev->name, "ks%i.%i", bus, cs);
    eth_register(dev);
#if defined(CONFIG_CMD_MII)
    miiphy_register(dev->name, ks_phy_read, ks_phy_write);
#endif
    return 1;
}

