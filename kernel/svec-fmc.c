/*
* Copyright (C) 2012-2013 CERN (www.cern.ch)
* Author: Juan David Gonzalez Cobas <dcobas@cern.ch>
* Author: Luis Fernando Ruiz Gago <lfruiz@cern.ch>
*
* Released according to the GNU GPL, version 2 or any later version
*
* Driver for SVEC (Simple VME FMC carrier) board.
*/
#include <linux/slab.h>
#include <linux/fmc.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/fmc-sdb.h>
#include <linux/platform_device.h>
#include "svec.h"

static int trtl_id = 1;
static int vic_id = 1;

static int svec_show_sdb;
module_param_named(show_sdb, svec_show_sdb, int, 0444);

/* The main role of this file is offering the fmc_operations for the svec */
static struct fmc_operations svec_fmc_operations;
static int svec_scan_cores(struct svec_dev *svec);

static uint32_t svec_readl(struct fmc_device *fmc, int offset)
{
	uint32_t val = 0;

	val = ioread32be(fmc->fpga_base + offset);

	return val;
}

static void svec_writel(struct fmc_device *fmc, uint32_t val, int offset)
{
	iowrite32be(val, fmc->fpga_base + offset);
}


static int svec_reprogram_raw(struct fmc_device *fmc, struct fmc_driver *drv,
			      void *gw, unsigned long len)
{
	struct svec_dev *svec = fmc->carrier_data;
	struct device *dev = fmc->hwdev;
	int ret;

	if (!gw || !len) {
		dev_err(dev, "Invalid firmware buffer - buf: %p len: %ld\n",
			gw, len);
		return -EINVAL;
	}

	if (!drv)
		dev_info(dev, "Carrier FPGA re-program\n");

	fmc_free_sdb_tree(fmc);

	fmc->flags &= ~FMC_DEVICE_HAS_GOLDEN;

	/* load the firmware */
	ret = svec_load_fpga(svec, gw, len);
	if (ret < 0) {
		dev_err(dev, "error %i programming firmware\n", ret);
		return ret;
	}

	/* Configure & activate CSR functions depending on chosen AM */
	svec_setup_csr(svec);

	fmc->flags |= FMC_DEVICE_HAS_CUSTOM;

	svec_scan_cores(svec);
	return 0;
}

static struct resource htvic_resource[] = {
	DEFINE_RES_MEM_NAMED(0x0,0x100, "base"),
	DEFINE_RES_IRQ_NAMED(0, "carrier"),
};
static struct platform_device htvic_device = {
	.name = "htvic-svec",
	.num_resources = ARRAY_SIZE(htvic_resource),
};

static void svec_release_vic(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);

	kfree(pdev->resource);
	kfree(pdev);
}

static int svec_create_vic(struct svec_dev *svec)
{
	struct vme_dev *vme_dev = to_vme_dev(svec->dev);
	struct platform_device *pdev;
	int ret, i;

	if (svec->pdev_vic)
		return -EBUSY;

	for (i = 0; i < SVEC_N_SLOTS; ++i) {
		ret = fmc_scan_sdb_tree(svec->fmcs[i], 0x0);
		if (ret >= 0 || ret == -EBUSY)
			break;
	}

	ret = fmc_find_sdb_device(svec->fmcs[0]->sdb, 0xCE42, 0x0013, NULL);
	if (ret < 0)
		return ret;

	pdev = kmemdup(&htvic_device, sizeof(struct platform_device), GFP_KERNEL);
	if (!pdev) {
		dev_err(svec->dev,
			"cannot allocate VIC device\n");
		return -ENOMEM;
	}
	pdev->dev.parent = &vme_dev->dev;
	pdev->dev.release = svec_release_vic;
	pdev->id = vic_id++;
	pdev->resource = kmemdup(htvic_resource,
				 sizeof(struct resource) * ARRAY_SIZE(htvic_resource),
				 GFP_KERNEL);
	if (!pdev->resource) {
		ret = -ENOMEM;
		goto out_res;
	}
	/* Set the FPGA base address and mockturtle base address */
	pdev->resource[0].parent = &svec->res[MAP_REG];
	pdev->resource[0].start = svec->res[MAP_REG].start + ret;
	pdev->resource[0].end = pdev->resource[0].start + 0x100 - 1;
	pdev->resource[1].start = vme_dev->irq;
	pdev->resource[1].flags |= IORESOURCE_IRQ_HIGHEDGE;

	ret = platform_device_register(pdev);
	if (ret)
		goto out_reg;

	svec->pdev_vic = pdev;

	return 0;

out_reg:
	kfree(pdev->resource);
out_res:
	kfree(pdev);

	return ret;
}

static void svec_destroy_vic(struct svec_dev *svec)
{
	if (!svec->pdev_vic)
		return;

	platform_device_unregister(svec->pdev_vic);
	svec->pdev_vic = NULL;
}

static struct resource trtl_resource[] = {
	DEFINE_RES_MEM_NAMED(0x0,0x20000, "base"),
	DEFINE_RES_IRQ_NAMED(0, "trtl-hmq"),
	DEFINE_RES_IRQ_NAMED(1, "trtl-dbg"),
};
static struct platform_device trtl_device = {
	.name = "mock-turtle-svec",
	.num_resources = ARRAY_SIZE(trtl_resource),
};

static void svec_release_trtl(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);

	kfree(pdev->resource);
	kfree(pdev);
}

static int svec_create_trtl(struct svec_dev *svec)
{
	struct vme_dev *vme_dev = to_vme_dev(svec->dev);
	struct platform_device *pdev;
	struct irq_domain *irqd;
	int ret, i;

	if (svec->pdev_trtl) {
		dev_err(svec->dev, "Mock Turtle aldready exists\n");
		return -EBUSY;
	}

	if (!svec->pdev_vic) {
		dev_err(svec->dev, "Mock Turtle needs the HTVIC device\n");
		return -ENODEV;
	}
	irqd = irq_find_host(svec->pdev_vic);
	if (!irqd) {
		dev_err(svec->dev, "Cannot find HT-VIC irq domain\n");
		return -ENODEV;
	}

	for (i = 0; i < SVEC_N_SLOTS; ++i) {
		ret = fmc_scan_sdb_tree(svec->fmcs[i], 0x0);
		if (ret >= 0 || ret == -EBUSY)
		        break;
	}

	/* Look for the WhiteRabbit NodeCore component and create a device
	   based of FMC device on virtual slot 2 */
	ret = fmc_find_sdb_device(svec->fmcs[i]->sdb, 0xCE42, 0x90DE, NULL);
	if (ret < 0)
		return ret;

	pdev = kmemdup(&trtl_device, sizeof(struct platform_device), GFP_KERNEL);
	if (!pdev) {
		dev_err(svec->dev,
			"cannot allocate Mock Turtle device\n");
		return -ENOMEM;
	}
	pdev->dev.parent = &vme_dev->dev;
	pdev->dev.release = svec_release_trtl;
	pdev->id = trtl_id++;
	pdev->resource = kmemdup(&trtl_resource,
				 sizeof(struct resource) * ARRAY_SIZE(trtl_resource),
				 GFP_KERNEL);
	if (!pdev->resource) {
		ret = -ENOMEM;
		goto out_res;
	}
	/* Set the FPGA base address and mockturtle base address */
	pdev->resource[0].parent = &svec->res[MAP_REG];
	pdev->resource[0].start = svec->res[MAP_REG].start + ret;
	pdev->resource[0].end = pdev->resource[0].start + 0x10000 - 1;
	/* Set mockturtle IRQ addresses */
	pdev->resource[1].start = irq_find_mapping(irqd, 2);
	pdev->resource[1].flags |= IORESOURCE_IRQ_HIGHEDGE;
	pdev->resource[2].start = irq_find_mapping(irqd, 3);
	pdev->resource[2].flags |= IORESOURCE_IRQ_HIGHEDGE;

	ret = platform_device_register(pdev);
	if (ret)
		goto out_reg;

	svec->pdev_trtl = pdev;

	return 0;

out_reg:
	kfree(pdev->resource);
out_res:
	kfree(pdev);
	return ret;
}

static void svec_destroy_trtl(struct svec_dev *svec)
{
	if (!svec->pdev_trtl)
		return;

	platform_device_unregister(svec->pdev_trtl);
	svec->pdev_trtl = NULL;
}

static int svec_scan_cores(struct svec_dev *svec)
{
	/* Destroy old components */
	/* Remove trtl before vic. MockTurtle use the VIC */
	svec_destroy_trtl(svec);
	svec_destroy_vic(svec);

	svec_create_vic(svec);
	svec_create_trtl(svec);

	return 0;
}

static int svec_reprogram(struct fmc_device *fmc, struct fmc_driver *drv,
			  char *gw)
{
	const struct firmware *fw;
	struct svec_dev *svec = fmc->carrier_data;
	struct device *dev = fmc->hwdev;
	int ret = 0;

	/* If no firmware filename is provided, load default */
	if (!gw)
		gw = svec_fw_name;

	if (!strlen(gw)) {	/* use module parameters from the driver */
		int index;

		/*
		 * FIXME: this should come from a call to validate.
		 * if the driver uses FMC_PARAM_BUSID/FMC_PARAM_GATEWARE.
		 * Please check docs and spec-sw (wr-nic).
		 */
		index = 0;

		gw = drv->gw_val[index];
		if (!gw)
			return -ESRCH;	/* the caller may accept this */
	}

	if (svec->verbose)
		dev_info(fmc->hwdev, "reprogramming with %s\n", gw);
	ret = request_firmware(&fw, gw, dev);
	if (ret < 0) {
		dev_warn(dev, "request firmware \"%s\": error %i\n", gw, ret);
		return ret;
	}

	/* Re-Program FPGA */
	ret = svec_reprogram_raw(fmc, drv, (void *)fw->data, fw->size);

	release_firmware(fw);
	if (ret < 0)
		dev_err(dev, "svec reprogram failed while loading %s\n", gw);

	return ret;
}

static int svec_validate(struct fmc_device *fmc, struct fmc_driver *drv)
{
	return 0;		/* everyhing is valid */
}

static int svec_gpio_config(struct fmc_device *fmc, struct fmc_gpio *gpio,
			    int ngpio)
{
	return 0;
}

static int svec_read_ee(struct fmc_device *fmc, int pos, void *data, int len)
{
	if (!(fmc->flags & FMC_DEVICE_HAS_GOLDEN))
		return -ENOTSUPP;
	return svec_eeprom_read(fmc, pos, data, len);
}

static int svec_write_ee(struct fmc_device *fmc, int pos,
			 const void *data, int len)
{
	if (!(fmc->flags & FMC_DEVICE_HAS_GOLDEN))
		return -ENOTSUPP;

	return svec_eeprom_write(fmc, pos, data, len);
}

static struct fmc_operations svec_fmc_operations = {
	.read32 = svec_readl,
	.write32 = svec_writel,
	.reprogram_raw = svec_reprogram_raw,
	.reprogram = svec_reprogram,
	.irq_request = NULL,
	.irq_ack = NULL,
	.irq_free = NULL,
	.gpio_config = svec_gpio_config,
	.read_ee = svec_read_ee,
	.write_ee = svec_write_ee,
	.validate = svec_validate,
};

static int check_golden(struct fmc_device *fmc)
{
	struct svec_dev *svec = fmc->carrier_data;
	int ret;
	uint32_t magic, vendor, device;

	/* poor man's SDB */
	magic = fmc_readl(fmc, 0x00);
	if (magic != 0x5344422d) {
		dev_err(svec->dev, "Bad SDB magic: 0x%08x\n", magic);
		return -ENODEV;
	}
	ret = fmc_scan_sdb_tree(fmc, 0x0);
	if (ret < 0)
		return -ENODEV;

	vendor = fmc_readl(fmc, 0x5c);
	if (vendor != 0x0000ce42) {
		dev_err(svec->dev, "unsexpected vendor in SDB\n");
		return -ENODEV;
	}
	device = fmc_readl(fmc, 0x60);
	if (device != 0x676f6c64) {
		dev_err(svec->dev, "unexpected device in SDB\n");
		return -ENODEV;
	}
	if (svec_show_sdb)
		fmc_show_sdb_tree(fmc);
	return 0;
}

int svec_fmc_prepare(struct svec_dev *svec, unsigned int fmc_slot)
{
	struct fmc_device *fmc;
	struct vme_dev *vme = to_vme_dev(svec->dev);
	int ret = 0;

	if (fmc_slot >= SVEC_N_SLOTS)
		return -EINVAL;

	fmc = kzalloc(sizeof(*fmc), GFP_KERNEL);
	if (!fmc) {
		dev_err(svec->dev, "cannot allocate fmc slot %d\n", fmc_slot);
		return -ENOMEM;
	}

	fmc->version = FMC_VERSION;
	fmc->carrier_name = "SVEC";
	fmc->carrier_data = svec;
	fmc->owner = THIS_MODULE;

	fmc->fpga_base = svec->map[MAP_REG]->kernel_va;

	fmc->irq = 0;		/*TO-DO */
	fmc->op = &svec_fmc_operations;
	fmc->hwdev = svec->dev;	/* for messages */

	fmc->slot_id = fmc_slot;
	fmc->device_id = (vme->slot << 6) | fmc_slot;
	fmc->eeprom_addr = 0x50 + 2 * fmc_slot;
	fmc->memlen = svec->cfg_cur.vme_size;

	/* check golden integrity */
	/* FIXME: this uses fmc_scan_sdb_tree and de-allocation
	 * could be wrong at second reprogramming, as it is called
	 * n times, one per slot */

	fmc->flags &= ~FMC_DEVICE_HAS_GOLDEN;
	fmc->flags &= ~FMC_DEVICE_HAS_CUSTOM;

	ret = svec_load_golden(svec);
	if (ret) {
		dev_err(svec->dev, "Cannot load golden bitstream: %d\n", ret);
		kfree(fmc);
		return ret;
	}

	ret = check_golden(fmc);
	if (ret) {
		dev_err(svec->dev, "Bad golden, error %d\n", ret);
		kfree(fmc);
		return ret;
	}

	fmc->flags |= FMC_DEVICE_HAS_GOLDEN;

	ret = svec_i2c_init(fmc);
	if (ret) {
		dev_err(svec->dev, "Error %d on svec i2c init", ret);
		kfree(fmc);
		return ret;
	}

	svec->fmcs[fmc_slot] = fmc;

	if (svec->verbose)
		dev_info(svec->dev, "ready to create fmc device_id 0x%x\n",
			 fmc->device_id);

	return ret;
}

int svec_fmc_create(struct svec_dev *svec, struct fmc_gateware *gw)
{
	int i;
	int error = 0;

	/* fmc structures filling */
	for (i = 0; i < svec->fmcs_n; i++) {
		error = svec_fmc_prepare(svec, i);
		if (error)
			goto failed;
	}

	/* fmc device creation */
	error = fmc_device_register_gw(svec->fmcs[0], gw);
	if (error) {
		dev_err(svec->dev, "Error registering fmc devices\n");
		goto failed;
	}

	/* fmc device creation */
	error = fmc_device_register(svec->fmcs[1]);
	if (error) {
		fmc_device_unregister(svec->fmcs[0]);
		dev_err(svec->dev, "Error registering fmc devices\n");
		goto failed;
	}

	/* FIXME: how do we retrieve the actual number of registered
	 * devices?
	 */
	if (svec->verbose)
		dev_info(svec->dev, "fmc devices registered\n");

	/* scan SDB. Do not report errors, we don't care, SDB is not
	   mandatory for the carrier board */
	for (i = 0; i < svec->fmcs_n; i++)
		fmc_scan_sdb_tree(svec->fmcs[i], 0);

	return 0;

failed:
	for (i = 0; i < svec->fmcs_n; i++)
		kfree(svec->fmcs[i]);

	/* FIXME: free fmc allocations. */
	return error;

}

void svec_fmc_destroy(struct svec_dev *svec)
{
	/* The MockTurtle use the VIC so remove it before */
	svec_destroy_trtl(svec);
	svec_destroy_vic(svec);

	if (!svec->fmcs[0])
		return;

	fmc_device_unregister_n(svec->fmcs, svec->fmcs_n);

	WARN(test_bit(SVEC_FLAG_IRQS_REQUESTED, &svec->flags),
	     "A Mezzanine driver didn't release all its IRQ handlers (FLAG 0x%lx)\n",
	     svec->flags);
	memset(svec->fmcs, 0, sizeof(struct fmc_devices *) * SVEC_N_SLOTS);

	if (svec->verbose)
		dev_info(svec->dev, "%d fmc devices unregistered\n",
			 svec->fmcs_n);

}
