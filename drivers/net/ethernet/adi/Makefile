# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for ADI Ethernet device drivers.
#

obj-$(CONFIG_ADRV906X_NET) += adrv906x-eth.o
adrv906x-eth-y := adrv906x-net.o adrv906x-mac.o adrv906x-switch.o adrv906x-ndma.o adrv906x-ethtool.o
adrv906x-eth-$(CONFIG_MACSEC) += adrv906x-macsec-ext.o macsec/cco_macsec.o
