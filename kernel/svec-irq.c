/*
* Copyright (C) 2013 CERN (www.cern.ch)
* Author: Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
*
* Released according to the GNU GPL, version 2 or any later version
*
* Driver for SVEC (Simple VME FMC carrier) board.
* Interrupt support code.
*/

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fmc.h>
#include <linux/fmc-sdb.h>
#include <linux/spinlock.h>
#include <htvic.h>
#include "vmebus.h"

#include "svec.h"

/* "master" SVEC interrupt handler */
static int svec_irq_handler(void *data)
{
	struct svec_dev *svec = (struct svec_dev *)data;
	int i;
	int rv = IRQ_HANDLED;
	unsigned long flags;

	svec->irq_count++;

	/*
	 * just in case we had an IRQ while messing around with the
	 * VIC registers/fmc_handlers
	 */
	spin_lock_irqsave(&svec->irq_lock, flags);

	if (svec_has_ht_vic(svec))
		rv = ht_vic_handler_irq(&svec->vic);
	else {
		/*
		 * shared irq mode: call all handlers until one of them
		 * has dealt with the interrupt
		 */
		for (i = 0; i < SVEC_N_SLOTS; i++) {
			irq_handler_t handler = svec->fmc_handlers[i];

			/*
			 * Call all handlers even if the current one
			 * returned IRQ_HANDLED. The SVEC VME Core IRQ
			 * is edge-sensitive, doing otherwise could
			 * result in missed irqs!
			 */
			if (handler)
				handler(i, svec->fmcs[i]);
		}
	}

	spin_unlock_irqrestore(&svec->irq_lock, flags);

	if (rv < 0) {
		dev_warn(svec->dev, "spurious VME interrupt, ignoring\n");
		return IRQ_HANDLED;
	}

	return rv;
}

int svec_irq_request(struct fmc_device *fmc, irq_handler_t handler,
		     char *name, int flags)
{
	struct svec_dev *svec = (struct svec_dev *)fmc->carrier_data;
	int rv = 0;

	/*
	 * Depending on IRQF_SHARED flag, choose between a VIC and
	 * shared IRQ mode
	 */
	if (!flags && svec_has_ht_vic(svec))
		rv = ht_vic_request_irq(&svec->vic, fmc, fmc->irq, handler);
	else if (flags & IRQF_SHARED) {
		spin_lock(&svec->irq_lock);
		svec->fmc_handlers[fmc->slot_id] = handler;
		spin_unlock(&svec->irq_lock);
	} else
		return -EINVAL;

	return rv;
}

void svec_irq_ack(struct fmc_device *fmc)
{
	/* Done by VIC module and VME bus */
}

int svec_irq_free(struct fmc_device *fmc)
{
	struct svec_dev *svec = (struct svec_dev *)fmc->carrier_data;

	/* freeing a nonexistent interrupt? */
	if (!test_bit(SVEC_FLAG_IRQS_REQUESTED, &svec->flags))
		return -EINVAL;

	if (svec_has_ht_vic(svec)) {
		ht_vic_free_irq(&svec->vic, fmc->irq);
	} else {
		spin_lock(&svec->irq_lock);
		svec->fmc_handlers[fmc->slot_id] = NULL;
		spin_unlock(&svec->irq_lock);
	}

	return 0;
}

int svec_irq_init(struct svec_dev *svec)
{
	int rv;

	rv = vme_request_irq(svec->cfg_cur.interrupt_vector,
			     svec_irq_handler, (void *)svec,
			     svec->name);
	svec->current_vector = svec->cfg_cur.interrupt_vector;

	return rv;
}

void svec_irq_exit(struct svec_dev *svec)
{
	int rv;

	rv = vme_free_irq(svec->current_vector);
	if (rv < 0)
		dev_err(svec->dev, "Error while freeing the VME irq (%d)\n",
			rv);
}


/* * * * VIC * * * */
/*
 * NOTE: with a real bus most of the following code will disapear becase
 * the matching and the registration will be done by the bus
 */
int svec_has_ht_vic(struct svec_dev *svec)
{
	return svec->vic.name != NULL;
}

static void spec_vic_release(struct device *dev){}

int svec_ht_vic_init(struct svec_dev *svec)
{
	struct fmc_device *fmc = NULL;
	signed long vic_base;
	int err, i;

	/*
	 * It does not matter if we use fmcs[0] for all device because
	 * we are going to parse SDB which is shared between all slosts
	 */
	for (i = 0; i < SVEC_N_SLOTS; ++i) {
		fmc = svec->fmcs[i];
		if (fmc)
			break;
	}
	if (!fmc) {
		dev_warn(svec->dev,
			 "no FMC devices. VIC interrupt will not work\n");
		return -ENODEV;
	}

	fmc_scan_sdb_tree(fmc, 0x0);
	/*
	 * Try to look up the VIC in the SDB tree - note that IRQs
	 * shall be requested after the FMC driver has scanned the SDB tree
	 */
	vic_base = fmc_find_sdb_device(fmc->sdb, HT_VIC_SDB_VENDOR,
				       HT_VIC_SDB_DEVICE, NULL);
	if (vic_base < 0)
		return -ENODEV;

	svec->vic.name = kasprintf(GFP_KERNEL, "ht-vic");
	svec->vic.dev.release = spec_vic_release;

	platform_set_drvdata(&svec->vic, fmc);
	err = platform_device_register(&svec->vic);
	if (err)
	        goto out_reg;

	return 0;

out_reg:
	put_device(&svec->vic.dev);
	kfree(svec->vic.name);
	/* clear kobject init status and restore structure */
	memset(&svec->vic, 0, sizeof(struct ht_vic_irq_chip));
	return err;
}

void spec_ht_vic_exit(struct svec_dev *svec)
{
	if (!svec_has_ht_vic(svec))
		return;

	kfree(svec->vic.name);
	platform_device_unregister(&svec->vic);
	put_device(&svec->vic.dev);
	/* clear kobject init status */
	memset(&svec->vic, 0, sizeof(struct ht_vic_irq_chip));
}

/* * * * * END of VIC * * * * */
