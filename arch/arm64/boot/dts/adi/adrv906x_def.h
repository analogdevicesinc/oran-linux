/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADI_ADRV906X_DEF_H__
#define __ADI_ADRV906X_DEF_H__

#define ANTENNA_CAL_DDE_BASE    0x20280000
#define DEBUG_DDE_INJ_BASE      0x20270000
#define EMAC_1G_BASE    0x20720000
#define EMAC_CMN_BASE   0x2B300000
#define EMAC_MACSEC_0_BASE      0x2B360000
#define EMAC_MACSEC_1_BASE      0x2B370000
#define EMAC_MAC_0_BASE 0x2B330000
#define EMAC_MAC_0_RX   0x2B331300
#define EMAC_MAC_0_TX   0x2B330300
#define EMAC_MAC_1_BASE 0x2B340000
#define EMAC_MAC_1_RX   0x2B341300
#define EMAC_MAC_1_TX   0x2B340300
#define EMAC_PCS_0_BASE 0x2B310000
#define EMAC_PCS_0_TSU  0x2B310400
#define EMAC_PCS_1_BASE 0x2B320000
#define EMAC_PCS_1_TSU  0x2B320400
#define EMAC_SW_BASE    0x2B350100
#define EMAC_SW_MAE_BASE        0x2B350300
#define EMAC_SW_PORT_0_BASE     0x2B350600
#define EMAC_SW_PORT_1_BASE     0x2B350D00
#define EMAC_SW_PORT_2_BASE     0x2B351400
#define EMAC_TOD_BASE   0x2B380000
#define EMMC_0_BASE     0x20724000
#define EMMC_0_PHY_BASE 0x20724300
#define GIC_BASE        0x21000000
#define GPIO_NS_BASE    0x2021B000
#define I2C_0_BASE      0x20760000
#define I2C_1_BASE      0x20760100
#define I2C_2_BASE      0x20760200
#define I2C_3_BASE      0x20760300
#define I2C_4_BASE      0x20760400
#define I2C_5_BASE      0x20760500
#define I2C_6_BASE      0x20760600
#define I2C_7_BASE      0x20760700
#define MDMA_0_CH00_BASE        0x20020000
#define NDMA_0_RX_BASE  0x20214000
#define NDMA_0_RX_DMA   0x20260000
#define NDMA_0_TX_BASE  0x20216000
#define NDMA_0_TX_DMA   0x20261000
#define NDMA_0_TX_STATUS_DMA    0x20264000
#define NDMA_1_RX_BASE  0x20215000
#define NDMA_1_RX_DMA   0x20262000
#define NDMA_1_TX_BASE  0x20217000
#define NDMA_1_TX_DMA   0x20263000
#define NDMA_1_TX_STATUS_DMA    0x20265000
#define PIMC_DDE_BASE   0x202A0000
#define PINCTRL_BASE    0x20218000
#define PINTMUX_BASE    0x20102200
#define PL011_0_BASE    0x20060000
#define PL011_1_BASE    0x20061000
#define PL011_2_BASE    0x20062000
#define PL011_3_BASE    0x20063000
#define PWM_BASE        0x20727000
#define QSPI_0_RX_DDE_BASE      0x20731000
#define QSPI_0_TX_DDE_BASE      0x20730000
#define QSPI_BASE       0x20732000
#define SD_0_BASE       0x20725000
#define SEC_EMAC_MACSEC_0_BASE  0x2F360000
#define SEC_EMAC_MACSEC_1_BASE  0x2F370000
#define SEC_GIC_BASE    0x25000000
#define SEC_GPIO_NS_BASE        0x2421B000
#define SEC_PINCTRL_BASE        0x24218000
#define SEC_PINTMUX_BASE        0x24102200
#define SEC_PL011_3_BASE        0x24063000
#define SERDES_0_RX_BASE        0x2B390000
#define SERDES_0_TX_BASE        0x2B392000
#define SERDES_1_RX_BASE        0x2B390800
#define SERDES_1_TX_BASE        0x2B392800
#define SERDES_4_PACK_BASE      0x2B398000
#define SPI_0_BASE      0x20733000
#define SPI_1_BASE      0x20734000
#define SPI_2_BASE      0x20735000
#define SPI_3_BASE      0x20736000
#define SPI_4_BASE      0x20737000
#define SPI_5_BASE      0x20738000
#define TRU_BASE        0x20052000
#define VIRTUAL_PL011_0_0_BASE  0x20064000
#define VIRTUAL_PL011_0_1_BASE  0x20065000
#define VIRTUAL_PL011_1_1_BASE  0x20067000

#define ANTENNA_CAL_DDE_BASE_UADDR      20280000
#define DEBUG_DDE_INJ_BASE_UADDR        20270000
#define EMAC_1G_BASE_UADDR      20720000
#define EMAC_CMN_BASE_UADDR     2B300000
#define EMAC_MACSEC_0_BASE_UADDR        2B360000
#define EMAC_MACSEC_1_BASE_UADDR        2B370000
#define EMAC_MAC_0_BASE_UADDR   2B330000
#define EMAC_MAC_0_RX_UADDR     2B331300
#define EMAC_MAC_0_TX_UADDR     2B330300
#define EMAC_MAC_1_BASE_UADDR   2B340000
#define EMAC_MAC_1_RX_UADDR     2B341300
#define EMAC_MAC_1_TX_UADDR     2B340300
#define EMAC_PCS_0_BASE_UADDR   2B310000
#define EMAC_PCS_0_TSU_UADDR    2B310400
#define EMAC_PCS_1_BASE_UADDR   2B320000
#define EMAC_PCS_1_TSU_UADDR    2B320400
#define EMAC_SW_BASE_UADDR      2B350100
#define EMAC_SW_MAE_BASE_UADDR  2B350300
#define EMAC_SW_PORT_0_BASE_UADDR       2B350600
#define EMAC_SW_PORT_1_BASE_UADDR       2B350D00
#define EMAC_SW_PORT_2_BASE_UADDR       2B351400
#define EMAC_TOD_BASE_UADDR     2B380000
#define EMMC_0_BASE_UADDR       20724000
#define EMMC_0_PHY_BASE_UADDR   20724300
#define GIC_BASE_UADDR  21000000
#define GPIO_NS_BASE_UADDR      2021B000
#define I2C_0_BASE_UADDR        20760000
#define I2C_1_BASE_UADDR        20760100
#define I2C_2_BASE_UADDR        20760200
#define I2C_3_BASE_UADDR        20760300
#define I2C_4_BASE_UADDR        20760400
#define I2C_5_BASE_UADDR        20760500
#define I2C_6_BASE_UADDR        20760600
#define I2C_7_BASE_UADDR        20760700
#define MDMA_0_CH00_BASE_UADDR  20020000
#define NDMA_0_RX_BASE_UADDR    20214000
#define NDMA_0_RX_DMA_UADDR     20260000
#define NDMA_0_TX_BASE_UADDR    20216000
#define NDMA_0_TX_DMA_UADDR     20261000
#define NDMA_0_TX_STATUS_DMA_UADDR      20264000
#define NDMA_1_RX_BASE_UADDR    20215000
#define NDMA_1_RX_DMA_UADDR     20262000
#define NDMA_1_TX_BASE_UADDR    20217000
#define NDMA_1_TX_DMA_UADDR     20263000
#define NDMA_1_TX_STATUS_DMA_UADDR      20265000
#define PIMC_DDE_BASE_UADDR     202A0000
#define PINCTRL_BASE_UADDR      20218000
#define PINTMUX_BASE_UADDR      20102200
#define PL011_0_BASE_UADDR      20060000
#define PL011_1_BASE_UADDR      20061000
#define PL011_2_BASE_UADDR      20062000
#define PL011_3_BASE_UADDR      20063000
#define PWM_BASE_UADDR  20727000
#define QSPI_0_RX_DDE_BASE_UADDR        20731000
#define QSPI_0_TX_DDE_BASE_UADDR        20730000
#define QSPI_BASE_UADDR 20732000
#define SD_0_BASE_UADDR 20725000
#define SEC_EMAC_MACSEC_0_BASE_UADDR    2F360000
#define SEC_EMAC_MACSEC_1_BASE_UADDR    2F370000
#define SEC_GIC_BASE_UADDR      25000000
#define SEC_GPIO_NS_BASE_UADDR  2421B000
#define SEC_PINCTRL_BASE_UADDR  24218000
#define SEC_PINTMUX_BASE_UADDR  24102200
#define SEC_PL011_3_BASE_UADDR  24063000
#define SERDES_0_RX_BASE_UADDR  2B390000
#define SERDES_0_TX_BASE_UADDR  2B392000
#define SERDES_1_RX_BASE_UADDR  2B390800
#define SERDES_1_TX_BASE_UADDR  2B392800
#define SERDES_4_PACK_BASE_UADDR        2B398000
#define SPI_0_BASE_UADDR        20733000
#define SPI_1_BASE_UADDR        20734000
#define SPI_2_BASE_UADDR        20735000
#define SPI_3_BASE_UADDR        20736000
#define SPI_4_BASE_UADDR        20737000
#define SPI_5_BASE_UADDR        20738000
#define TRU_BASE_UADDR  20052000
#define VIRTUAL_PL011_0_0_BASE_UADDR    20064000
#define VIRTUAL_PL011_0_1_BASE_UADDR    20065000
#define VIRTUAL_PL011_1_1_BASE_UADDR    20067000

#define EMAC_1G_DIV_CTRL    0x201c0050
#define EMAC_1G_CLK_CTRL    0x20190000
#define NDMA_RST_CTRL    0x201c0000
#define NDMA_0_INTR_CTRL    0x201c0060
#define NDMA_1_INTR_CTRL    0x201c0064
#define OIF_0_RX_CTRL    0x2b103040
#define OIF_0_TX_CTRL    0x2b10903c
#define OIF_1_RX_CTRL    0x2b104340
#define OIF_1_TX_CTRL    0x2b10b83c

#define EMAC_1G_DIV_CTRL_UADDR    201C0050
#define EMAC_1G_CLK_CTRL_UADDR    20190000
#define NDMA_RST_CTRL_UADDR    201C0000
#define NDMA_0_INTR_CTRL_UADDR    201C0060
#define NDMA_1_INTR_CTRL_UADDR    201C0064
#define OIF_0_RX_CTRL_UADDR    2B103040
#define OIF_0_TX_CTRL_UADDR    2B10903C
#define OIF_1_RX_CTRL_UADDR    2B104340
#define OIF_1_TX_CTRL_UADDR    2B10B83C


#endif /* __ADI_ADRV906X_DEF_H__ */
