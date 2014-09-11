/*******************************************************************************
  This contains the functions to handle the pci driver.

  Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/pci.h>
#include <linux/dmi.h>
#include "stmmac.h"

static void quark_run_time_config(int chip_id, struct pci_dev *pdev);

enum chip {
	CHIP_STMICRO = 0,
	CHIP_QUARK_X1000,
};

/* A struct for platform specific information which is used
 * in stmmac_default_data function for initialization
 */
struct platform_data {
	int phy_addr;
	int interface;
	int clk_csr;
	int has_gmac;
	int force_sf_dma_mode;
	int multicast_filter_bins;
	int unicast_filter_entries;
	int (*phy_reset)(void *priv);
	unsigned int phy_mask;
	int pbl;
	int fixed_burst;
	int burst_len;
	void (*rt_config)(int chip_id, struct pci_dev *pdev);
};

static struct platform_data platform_info[] = {
	[CHIP_STMICRO] = {
		.phy_addr = 0,
		.interface = PHY_INTERFACE_MODE_GMII,
		.clk_csr = 2,
		.has_gmac = 1,
		.force_sf_dma_mode = 1,
		.multicast_filter_bins = HASH_TABLE_SIZE,
		.unicast_filter_entries = 1,
		.phy_reset = NULL,
		.phy_mask = 0,
		.pbl = 32,
		.fixed_burst = 0,
		.burst_len = DMA_AXI_BLEN_256,
		.rt_config = NULL,
	},
	[CHIP_QUARK_X1000] = {
		.phy_addr = 1,
		.interface = PHY_INTERFACE_MODE_RMII,
		.clk_csr = 2,
		.has_gmac = 1,
		.force_sf_dma_mode = 1,
		.multicast_filter_bins = HASH_TABLE_SIZE,
		.unicast_filter_entries = 1,
		.phy_reset = NULL,
		.phy_mask = 0,
		.pbl = 16,
		.fixed_burst = 1,
		.burst_len = DMA_AXI_BLEN_256,
		.rt_config = &quark_run_time_config,
	},
};

/* This struct is used to associate PCI Function ID of MAC controller
 * on a board, discovered via DMI, with phy_address. It is also used
 * to describe if that MAC controller is connected with PHY.
 */
struct intel_quark_platform {
	int pci_func_num;
	const char *board_name;
	int phy_address;
};

static struct intel_quark_platform quark_x1000_phy_info[] = {
	{
		.pci_func_num = 7,
		.board_name = "Galileo",
		/* Galileo ethernet port 2 does not connect to any PHY */
		.phy_address = -1,
	},
	{
		.pci_func_num = 7,
		.board_name = "GalileoGen2",
		/* Galileo Gen2 ethernet port 2 does not connect to any PHY */
		.phy_address = -1,
	},
};

static void quark_run_time_config(int chip_id, struct pci_dev *pdev)
{
	const char *board_name = dmi_get_system_info(DMI_BOARD_NAME);
	int i;
	int func_num = PCI_FUNC(pdev->devfn);

	if (!board_name)
		return;

	for (i = 0; i < ARRAY_SIZE(quark_x1000_phy_info); i++) {
		if ((!strcmp(quark_x1000_phy_info[i].board_name, board_name)) &&
		    quark_x1000_phy_info[i].pci_func_num == func_num)
			platform_info[chip_id].phy_addr =
				quark_x1000_phy_info[i].phy_address;
	}
}

static int stmmac_default_data(struct plat_stmmacenet_data *plat,
			       int chip_id, struct pci_dev *pdev)
{
	struct platform_data *chip_plat_dat = &platform_info[chip_id];

	if (chip_plat_dat->rt_config)
		chip_plat_dat->rt_config(chip_id, pdev);
	plat->bus_id = PCI_DEVID(pdev->bus->number, pdev->devfn);
	plat->phy_addr = chip_plat_dat->phy_addr;
	plat->interface = chip_plat_dat->interface;
	/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */
	plat->clk_csr = chip_plat_dat->clk_csr;
	plat->has_gmac = chip_plat_dat->has_gmac;
	plat->force_sf_dma_mode = chip_plat_dat->force_sf_dma_mode;
	plat->multicast_filter_bins = chip_plat_dat->multicast_filter_bins;
	plat->unicast_filter_entries = chip_plat_dat->unicast_filter_entries;

	plat->mdio_bus_data->phy_reset = chip_plat_dat->phy_reset;
	plat->mdio_bus_data->phy_mask = chip_plat_dat->phy_mask;

	plat->dma_cfg->pbl = chip_plat_dat->pbl;
	plat->dma_cfg->fixed_burst = chip_plat_dat->fixed_burst;
	plat->dma_cfg->burst_len = chip_plat_dat->burst_len;

	/* Refuse to load the driver and register net device
	 * if MAC controller does not connect to any PHY interface
	 */
	return (plat->phy_addr != -1) ? 0 : -ENODEV;
}

/**
 * stmmac_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int stmmac_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	int ret = 0;
	void __iomem *addr = NULL;
	struct stmmac_priv *priv = NULL;
	struct plat_stmmacenet_data *plat_dat;
	int i;

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		pr_err("%s : ERROR: failed to enable %s device\n", __func__,
		       pci_name(pdev));
		return ret;
	}
	if (pci_request_regions(pdev, STMMAC_RESOURCE_NAME)) {
		pr_err("%s: ERROR: failed to get PCI region\n", __func__);
		ret = -ENODEV;
		goto err_out_req_reg_failed;
	}

	/* Get the base address of device */
	for (i = 0; i <= 5; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		addr = pci_iomap(pdev, i, 0);
		if (addr == NULL) {
			pr_err("%s: ERROR: cannot map register memory aborting",
			       __func__);
			ret = -EIO;
			goto err_out_map_failed;
		}
		break;
	}
	pci_set_master(pdev);

	plat_dat = devm_kzalloc(&pdev->dev, sizeof(*plat_dat), GFP_KERNEL);
	if (!plat_dat) {
		ret = -ENOMEM;
		goto err_out;
	}

	plat_dat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					       sizeof(*plat_dat->mdio_bus_data),
					       GFP_KERNEL);
	if (!plat_dat->mdio_bus_data) {
		ret = -ENOMEM;
		goto err_out;
	}

	plat_dat->dma_cfg = devm_kzalloc(&pdev->dev,
					 sizeof(*plat_dat->dma_cfg),
					 GFP_KERNEL);
	if (!plat_dat->dma_cfg) {
		ret = -ENOMEM;
		goto err_out;
	}

	ret = stmmac_default_data(plat_dat, id->driver_data, pdev);
	if (ret)
		goto err_out;

	priv = stmmac_dvr_probe(&pdev->dev, plat_dat, addr);
	if (IS_ERR(priv)) {
		pr_err("%s: main driver probe failed", __func__);
		ret = PTR_ERR(priv);
		goto err_out;
	}
	priv->dev->irq = pdev->irq;
	priv->wol_irq = pdev->irq;

	pci_set_drvdata(pdev, priv->dev);

	pr_debug("STMMAC platform driver registration completed");

	return 0;

err_out:
	pci_clear_master(pdev);
err_out_map_failed:
	pci_release_regions(pdev);
err_out_req_reg_failed:
	pci_disable_device(pdev);

	return ret;
}

/**
 * stmmac_pci_remove
 *
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and releases the PCI resources.
 */
static void stmmac_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	stmmac_dvr_remove(ndev);

	pci_iounmap(pdev, priv->ioaddr);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM
static int stmmac_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	int ret;

	ret = stmmac_suspend(ndev);
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return ret;
}

static int stmmac_pci_resume(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	return stmmac_resume(ndev);
}
#endif

#define STMMAC_VENDOR_ID 0x700
#define STMMAC_DEVICE_ID 0x1108
#define STMMAC_QUARK_X1000_ID 0x0937

static const struct pci_device_id stmmac_id_table[] = {
	{PCI_DEVICE(STMMAC_VENDOR_ID, STMMAC_DEVICE_ID), PCI_ANY_ID,
			PCI_ANY_ID, CHIP_STMICRO},
	{PCI_VDEVICE(STMICRO, PCI_DEVICE_ID_STMICRO_MAC), CHIP_STMICRO},
	{PCI_VDEVICE(INTEL, STMMAC_QUARK_X1000_ID), CHIP_QUARK_X1000},
	{}
};

MODULE_DEVICE_TABLE(pci, stmmac_id_table);

struct pci_driver stmmac_pci_driver = {
	.name = STMMAC_RESOURCE_NAME,
	.id_table = stmmac_id_table,
	.probe = stmmac_pci_probe,
	.remove = stmmac_pci_remove,
#ifdef CONFIG_PM
	.suspend = stmmac_pci_suspend,
	.resume = stmmac_pci_resume,
#endif
};

MODULE_DESCRIPTION("STMMAC 10/100/1000 Ethernet PCI driver");
MODULE_AUTHOR("Rayagond Kokatanur <rayagond.kokatanur@vayavyalabs.com>");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");
