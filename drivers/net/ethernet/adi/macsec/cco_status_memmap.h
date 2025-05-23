// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2025, Analog Devices Incorporated, All Rights Reserved
 */
/******************************************************************************/
/*                                DO NOT MODIFY                               */
/*           THIS FILE IS AUTOGENERATED AND ALL CHANGES WILL BE LOST          */
/******************************************************************************/
#ifndef _MACSEC_TOP_STATUS_MEMMAP_H_
#define _MACSEC_TOP_STATUS_MEMMAP_H_

/* Status for controlled port, Tx/Rx SC/SA */
#define STATUS_BASE_ADDR                                                                                      0x00000900
#define STATUS_STRIDE                                                                                         0x00000100
/******************************************************************************/
/* Control for status readback */
#define STATUS_ST_CTRL_BASE_ADDR                                                                              0x00000000
/* RD_TRIGGER W1C Trigger a read operation */
#define STATUS_ST_CTRL_RD_TRIGGER_MASK                                                                        0x00001000
#define STATUS_ST_CTRL_RD_TRIGGER_SHIFT                                                                               12

/* SA_INDEX RW Index number of SA */
#define STATUS_ST_CTRL_SA_INDEX_MASK                                                                          0x00000c00
#define STATUS_ST_CTRL_SA_INDEX_SHIFT                                                                                 10

/* PEER_INDEX RW Index number of peer */
#define STATUS_ST_CTRL_PEER_INDEX_MASK                                                                        0x000003f0
#define STATUS_ST_CTRL_PEER_INDEX_SHIFT                                                                                4

/* SECY_INDEX RW Index number of SecY */
#define STATUS_ST_CTRL_SECY_INDEX_MASK                                                                        0x0000000f
#define STATUS_ST_CTRL_SECY_INDEX_SHIFT                                                                                0

/******************************************************************************/
/* Provided Interface registers:
 * Controlled port status - SecY status of:
 * 1) MAC_Enabled
 * 2) MAC_Operational */
#define STATUS_ST_SECY_STATUS_BASE_ADDR                                                                       0x00000004
/* MAC_OPERATIONAL RO True if and only if
 * 1) MAC_Enabled is True, and
 * 2) MAC_Operational is True for the Common Port. */
#define STATUS_ST_SECY_STATUS_MAC_OPERATIONAL_MASK                                                            0x00000002
#define STATUS_ST_SECY_STATUS_MAC_OPERATIONAL_SHIFT                                                                    1

/* MAC_ENABLED RO True if and only if
 * 1) ControlledPortEnabled (10.7.5) is True, and
 * 2) MAC_Enabled is True for the Common Port, and
 * 3) transmitting (10.7.21) is True for the transmit SC, and
 * 4) receiving (10.7.12) is True for at least one receive SC. */
#define STATUS_ST_SECY_STATUS_MAC_ENABLED_MASK                                                                0x00000001
#define STATUS_ST_SECY_STATUS_MAC_ENABLED_SHIFT                                                                        0

/******************************************************************************/
/* Transmit SC status */
#define STATUS_ST_TX_SC_STATUS_BASE_ADDR                                                                      0x00000008
/* INCLUDINGSCI RO Frame generation controls:
 * True if and only if the SC bit is set and the SCI explicitly
 * encoded in each SecTAG transmitted */
#define STATUS_ST_TX_SC_STATUS_INCLUDINGSCI_MASK                                                              0x00000008
#define STATUS_ST_TX_SC_STATUS_INCLUDINGSCI_SHIFT                                                                      3

/* ENCODINGSA RO Transmit SC status:
 * SA inUse at any time */
#define STATUS_ST_TX_SC_STATUS_ENCODINGSA_MASK                                                                0x00000006
#define STATUS_ST_TX_SC_STATUS_ENCODINGSA_SHIFT                                                                        1

/* TRANSMITTING RO Transmit SC status:
 * True if inUse (10.7.23) is True for any of the SAs for the SC,
 * and False otherwise */
#define STATUS_ST_TX_SC_STATUS_TRANSMITTING_MASK                                                              0x00000001
#define STATUS_ST_TX_SC_STATUS_TRANSMITTING_SHIFT                                                                      0

/******************************************************************************/
/* Transmit SA controls configuration */
#define STATUS_ST_TX_SA_STATUS_BASE_ADDR                                                                      0x0000000c
/* INUSE RO Transmit SA status:
 * If inUse is True, and MAC_Operational is True for the Common Port,
 * the SA can transmit frames */
#define STATUS_ST_TX_SA_STATUS_INUSE_MASK                                                                     0x00000001
#define STATUS_ST_TX_SA_STATUS_INUSE_SHIFT                                                                             0

/******************************************************************************/
/* Transmit SA status:
 * the current value of Transmit PN (10.5.2) for the SA */
#define STATUS_ST_TX_SA_NEXT_PN_BASE_ADDR                                                                     0x00000010
/* VAL RO ---- */
#define STATUS_ST_TX_SA_NEXT_PN_VAL_MASK                                                                      0xffffffff
#define STATUS_ST_TX_SA_NEXT_PN_VAL_SHIFT                                                                              0

/******************************************************************************/
/* Receive SC status and configuration */
#define STATUS_ST_RX_SC_STATUS_BASE_ADDR                                                                      0x00000014
/* RECEIVING RO Receive SC status:
 * True if inUse (10.7.14) is True for any of the SAs for the SC,
 * and False otherwise.
 * When the SC is created, receiving is False, and startedTime
 * and stoppedTime are equal to createdTime. */
#define STATUS_ST_RX_SC_STATUS_RECEIVING_MASK                                                                 0x00000001
#define STATUS_ST_RX_SC_STATUS_RECEIVING_SHIFT                                                                         0

/******************************************************************************/
/* Receive SA status and configuration */
#define STATUS_ST_RX_SA_STATUS_BASE_ADDR                                                                      0x00000018
/* INUSE RO Receive SA status */
#define STATUS_ST_RX_SA_STATUS_INUSE_MASK                                                                     0x00000001
#define STATUS_ST_RX_SA_STATUS_INUSE_SHIFT                                                                             0

/******************************************************************************/
/* Receive SA status:
 * Current value of nextPN */
#define STATUS_ST_RX_SA_NEXTPN_BASE_ADDR                                                                      0x0000001c
/* VAL RO ---- */
#define STATUS_ST_RX_SA_NEXTPN_VAL_MASK                                                                       0xffffffff
#define STATUS_ST_RX_SA_NEXTPN_VAL_SHIFT                                                                               0

/******************************************************************************/
/* Receive SA status:
 * Lowest acceptable PN value for a received frame */
#define STATUS_ST_RX_SA_LOWESTPN_BASE_ADDR                                                                    0x00000020
/* VAL RO ---- */
#define STATUS_ST_RX_SA_LOWESTPN_VAL_MASK                                                                     0xffffffff
#define STATUS_ST_RX_SA_LOWESTPN_VAL_SHIFT                                                                             0

#endif /* _MACSEC_TOP_STATUS_MEMMAP_H_ */
