/*
 * Copyright (C) 2016 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 *
 * Released according to the GNU GPL, version 2 or any later version
 */

#ifndef __DDM_H__
#define __DDM_H__

#include <linux/ioport.h>
#include "ddm-user.h"

struct ddm_resource {
	struct ddm_resource_id id;
	struct device *parent;
	struct resource *res;

	/* used by the DDM core */
	struct list_head list;
};

extern int ddm_resource_add(struct ddm_resource *res);
extern int ddm_resource_del(struct ddm_resource *res);
#endif
