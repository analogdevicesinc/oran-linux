// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2025, Analog Devices Incorporated, All Rights Reserved
 */
/******************************************************************************/
/*                                DO NOT MODIFY                               */
/*           THIS FILE IS AUTOGENERATED AND ALL CHANGES WILL BE LOST          */
/******************************************************************************/
#ifndef _MACSEC_TOP_TRANSMITSC_MEMMAP_H_
#define _MACSEC_TOP_TRANSMITSC_MEMMAP_H_

/* transmitChannels */
#define TRANSMITSC_BASE_ADDR                                                                                  0x00000300
#define TRANSMITSC_STRIDE                                                                                     0x00000100
/******************************************************************************/
/* Control and status indicator for rule / configuration insert */
#define TRANSMITSC_TX_SC_CTRL_BASE_ADDR                                                                       0x00000000
/* RD_TRIGGER W1C Trigger a read operation */
#define TRANSMITSC_TX_SC_CTRL_RD_TRIGGER_MASK                                                                 0x00000020
#define TRANSMITSC_TX_SC_CTRL_RD_TRIGGER_SHIFT                                                                         5

/* WR_TRIGGER W1C Trigger a write operation */
#define TRANSMITSC_TX_SC_CTRL_WR_TRIGGER_MASK                                                                 0x00000010
#define TRANSMITSC_TX_SC_CTRL_WR_TRIGGER_SHIFT                                                                         4

/* SECY_INDEX RW Index number of SecY to read/write SCI */
#define TRANSMITSC_TX_SC_CTRL_SECY_INDEX_MASK                                                                 0x0000000f
#define TRANSMITSC_TX_SC_CTRL_SECY_INDEX_SHIFT                                                                         0

/******************************************************************************/
/* Transmit SC ID to insert
 * When the SC is created, transmitting is False and startedTime
 * and stoppedTime are equal to createdTime
 * 31 downto 0 */
#define TRANSMITSC_TX_SC_SCI_0_WR_BASE_ADDR                                                                   0x00000004
/* VAL RW ---- */
#define TRANSMITSC_TX_SC_SCI_0_WR_VAL_MASK                                                                    0xffffffff
#define TRANSMITSC_TX_SC_SCI_0_WR_VAL_SHIFT                                                                            0

/******************************************************************************/
/* Transmit SC ID to insert
 * When the SC is created, transmitting is False and startedTime
 * and stoppedTime are equal to createdTime
 * 63 downto 32 */
#define TRANSMITSC_TX_SC_SCI_1_WR_BASE_ADDR                                                                   0x00000008
/* VAL RW ---- */
#define TRANSMITSC_TX_SC_SCI_1_WR_VAL_MASK                                                                    0xffffffff
#define TRANSMITSC_TX_SC_SCI_1_WR_VAL_SHIFT                                                                            0

/******************************************************************************/
/* Transmit SC ID to read
 * When the SC is created, transmitting is False and startedTime
 * and stoppedTime are equal to createdTime
 * 31 downto 0 */
#define TRANSMITSC_TX_SC_SCI_0_RD_BASE_ADDR                                                                   0x0000000c
/* VAL RO ---- */
#define TRANSMITSC_TX_SC_SCI_0_RD_VAL_MASK                                                                    0xffffffff
#define TRANSMITSC_TX_SC_SCI_0_RD_VAL_SHIFT                                                                            0

/******************************************************************************/
/* Transmit SC ID to read
 * When the SC is created, transmitting is False and startedTime
 * and stoppedTime are equal to createdTime
 * 63 downto 32 */
#define TRANSMITSC_TX_SC_SCI_1_RD_BASE_ADDR                                                                   0x00000010
/* VAL RO ---- */
#define TRANSMITSC_TX_SC_SCI_1_RD_VAL_MASK                                                                    0xffffffff
#define TRANSMITSC_TX_SC_SCI_1_RD_VAL_SHIFT                                                                            0

/******************************************************************************/
/* Transmit SC configuration */
#define TRANSMITSC_TX_SC_TRANSMITSC_CFG_BASE_ADDR                                                             0x00000014
/* ENABLETRANSMIT_SA_RD RO Transmit SC control: SA in Use at any time
 * When the SA is created, enableTransmit and inUse are False,
 * and the SA is not used to transmit frames. The SC parameter
 * encodingSA shall be set to the value of the AN for the SA
 * and inUse set True, when enableTransmit is set. The SA shall
 * stop transmitting, and inUse reset, when enableTransmit is reset
 * [0] - SA 0
 * [1] - SA 1
 * [2] - SA 2
 * [3] - SA 3 */
#define TRANSMITSC_TX_SC_TRANSMITSC_CFG_ENABLETRANSMIT_SA_RD_MASK                                             0x000000f0
#define TRANSMITSC_TX_SC_TRANSMITSC_CFG_ENABLETRANSMIT_SA_RD_SHIFT                                                     4

/* ENABLETRANSMIT_SA_WR RW Transmit SC control: SA in Use at any time
 * When the SA is created, enableTransmit and inUse are False,
 * and the SA is not used to transmit frames. The SC parameter
 * encodingSA shall be set to the value of the AN for the SA
 * and inUse set True, when enableTransmit is set. The SA shall
 * stop transmitting, and inUse reset, when enableTransmit is reset
 * [0] - SA 0
 * [1] - SA 1
 * [2] - SA 2
 * [3] - SA 3 */
#define TRANSMITSC_TX_SC_TRANSMITSC_CFG_ENABLETRANSMIT_SA_WR_MASK                                             0x0000000f
#define TRANSMITSC_TX_SC_TRANSMITSC_CFG_ENABLETRANSMIT_SA_WR_SHIFT                                                     0

#endif /* _MACSEC_TOP_TRANSMITSC_MEMMAP_H_ */
