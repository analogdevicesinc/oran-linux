// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include "adrv906x-mdio.h"
#include "adrv906x-net.h"

struct adrv906x_mdio_priv {
	struct device *dev;
	void __iomem *pcs_base[MAX_NETDEV_NUM];
};

static int adrv906x_pseudo_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 val)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;

	offset = 4 * (regnum & 0xFFFF);

	iowrite32(val, priv->pcs_base[mii_id % MAX_NETDEV_NUM] + offset);

	return 0;
}

static int adrv906x_pseudo_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;
	int ret;

	offset = 4 * (regnum & 0xFFFF);

	ret = ioread32(priv->pcs_base[mii_id % MAX_NETDEV_NUM] + offset) & 0xFFFF;

	return ret;
}

static int adrv906x_pseudo_mdio_write_c45(struct mii_bus *bus, int addr,
					  int devnum, int regnum, u16 val)
{
	return adrv906x_pseudo_mdio_write(bus, addr, regnum, val);
}

static int adrv906x_pseudo_mdio_read_c45(struct mii_bus *bus, int addr,
					 int devnum, int regnum)
{
	return adrv906x_pseudo_mdio_read(bus, addr, regnum);
}

int adrv906x_mdio_register(struct adrv906x_eth_if *eth_if, struct device_node *mdio_np)
{
	struct device *dev = eth_if->dev;
	struct adrv906x_mdio_priv *priv;
	struct mii_bus *bus;
	int idx, ret;
	u32 reg, len;

	bus = devm_mdiobus_alloc_size(dev, sizeof(*priv));
	if (!bus) {
		dev_err(dev, "failed to allocate private driver data");
		return -ENOMEM;
	}

	priv = bus->priv;
	priv->dev = dev;

	for (idx = 0; idx < MAX_NETDEV_NUM; idx++) {
		of_property_read_u32_index(mdio_np, "reg", 2 * idx, &reg);
		of_property_read_u32_index(mdio_np, "reg", 2 * idx + 1, &len);
		priv->pcs_base[idx] = devm_ioremap(dev, reg, len);

		if (IS_ERR(priv->pcs_base[idx]))
			return PTR_ERR(priv->pcs_base[idx]);
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));
	bus->name = "adrv906x-pseudo-mdio";
	bus->read = adrv906x_pseudo_mdio_read,
	bus->write = adrv906x_pseudo_mdio_write,
	bus->read_c45 = adrv906x_pseudo_mdio_read_c45,
	bus->write_c45 = adrv906x_pseudo_mdio_write_c45,
	bus->parent = dev;

	ret = of_mdiobus_register(bus, mdio_np);
	if (ret) {
		dev_err(dev, "failed to register mdio bus");
		return ret;
	}

	eth_if->mdio = bus;

	return 0;
}

void adrv906x_mdio_unregister(struct adrv906x_eth_if *eth_if)
{
	if (eth_if->mdio)
		mdiobus_unregister(eth_if->mdio);
}
