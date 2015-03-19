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
#include <linux/module.h>
#include <linux/fmc-sdb.h>
#include "svec.h"

static int svec_show_sdb;
module_param_named(show_sdb, svec_show_sdb, int, 0444);

/* The main role of this file is offering the fmc_operations for the svec */
static struct fmc_operations svec_fmc_operations;

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
	return 0;
}

#define WRNC_EEPROM_SIZE		8192	/* The standard eeprom size */
static const char wrnc_eeimg[WRNC_EEPROM_SIZE] = {
	0x01, 0x00, 0x00, 0x01, 0x00, 0x0a, 0x00, 0xf4, 0x01, 0x09, 0x00, 0xd9,
	0x99, 0x97, 0xc4, 0x63, 0x65, 0x72, 0x6e, 0xcc, 0x77, 0x72, 0x2d, 0x6e,
	0x6f, 0x64, 0x65, 0x2d, 0x63, 0x6f, 0x72, 0x65, 0xc4, 0x30, 0x30, 0x30,
	0x31, 0xcb, 0x73, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x2d, 0x70, 0x61, 0x72,
	0x74, 0xda, 0x32, 0x30, 0x31, 0x34, 0x2d, 0x31, 0x31, 0x2d, 0x32, 0x31,
	0x20, 0x31, 0x32, 0x3a, 0x34, 0x31, 0x3a, 0x33, 0x37, 0x2e, 0x37, 0x31,
	0x31, 0x39, 0x31, 0x31, 0xc1, 0x00, 0x00, 0xc4, 0x02, 0x02, 0x0d, 0xf7,
	0xf8, 0x02, 0xb0, 0x04, 0x74, 0x04, 0xec, 0x04, 0x00, 0x00, 0x00, 0x00,
	0xe8, 0x03, 0x02, 0x02, 0x0d, 0x5c, 0x93, 0x01, 0x4a, 0x01, 0x39, 0x01,
	0x5a, 0x01, 0x00, 0x00, 0x00, 0x00, 0xb8, 0x0b, 0x02, 0x02, 0x0d, 0x63,
	0x8c, 0x00, 0xfa, 0x00, 0xed, 0x00, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00,
	0xa0, 0x0f, 0x01, 0x02, 0x0d, 0xfb, 0xf5, 0x05, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x0d, 0xfc,
	0xf4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x02, 0x0d, 0xfd, 0xf3, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfa, 0x82, 0x0b, 0xea,
	0x8f, 0xa2, 0x12, 0x00, 0x00, 0x1e, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

static int svec_create_wrnc(struct svec_dev *svec)
{
	struct fmc_device *fmc;
	int ret, err, i;

	if (svec->fmc_wrnc) {
		dev_err(svec->dev, "WhiteRebbit NodeCore aldready exists\n");
		return -EBUSY;
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

	fmc = kzalloc(sizeof(*fmc), GFP_KERNEL);
	if (!fmc) {
		dev_err(svec->dev,
			"cannot allocate fmc slot for White-Rabbit Node-Core\n");
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

	fmc->slot_id = SVEC_N_SLOTS; /* Yes, is a virtual FMC slot, so
					outside SVEC slot enumeration */
	fmc->device_id = (svec->slot << 6) | fmc->slot_id;

	/* Overwrite with the white rabbit fake eeprom */
	fmc->eeprom_len = WRNC_EEPROM_SIZE;
	fmc->eeprom = wrnc_eeimg;
	fmc->eeprom_addr = 0x50 + 2 * fmc->slot_id;
	fmc->memlen = svec->cfg_cur.vme_size;

	fmc->flags &= ~FMC_DEVICE_HAS_GOLDEN;
	fmc->flags |= FMC_DEVICE_HAS_CUSTOM;

	err = fmc_device_register(fmc);
	if (err){
		dev_err(svec->dev, "Cannot register White-Rabbit Node-Core\n");
		return err;
	}

	svec->fmc_wrnc = fmc;

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
	.irq_request = svec_irq_request,
	.irq_ack = svec_irq_ack,
	.irq_free = svec_irq_free,
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
	fmc->device_id = (svec->slot << 6) | fmc_slot;
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
	error = fmc_device_register_n_gw(svec->fmcs, svec->fmcs_n, gw);
	if (error) {
		dev_err(svec->dev, "Error registering fmc devices\n");
		goto failed;
	}

	svec_create_wrnc(svec);

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
	if (svec->fmc_wrnc)
		fmc_device_unregister(svec->fmc_wrnc);

	if (!svec->fmcs[0])
		return;

	fmc_device_unregister_n(svec->fmcs, svec->fmcs_n);

	WARN(test_bit(SVEC_FLAG_IRQS_REQUESTED, &svec->flags) || svec->vic,
	     "A Mezzanine driver didn't release all its IRQ handlers (VIC %p, FLAG 0x%lx)\n",
	     svec->vic, svec->flags);
	memset(svec->fmcs, 0, sizeof(struct fmc_devices *) * SVEC_N_SLOTS);

	if (svec->verbose)
		dev_info(svec->dev, "%d fmc devices unregistered\n",
			 svec->fmcs_n);

}
