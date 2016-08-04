/*
 * Copyright (C) 2016 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 *
 * Released according to the GNU GPL, version 2 or any later version
 */

#ifndef __DDM_USER_H__
#define __DDM_USER_H__

#ifndef __KERNEL__
#include <stdint.h>
#endif

#define DDM_REGISTRATION_NAME_LEN 20 /* same size as PLATFORM_NAME_SIZE */
#define DDM_REGISTRATION_RES_MAX_LEN 32
#define DDM_REGISTRATION_PRIV_MAX_LEN 32

/**
 * DDM resource identifier
 */
struct ddm_resource_id {
	const char name[DDM_REGISTRATION_NAME_LEN];
	uint32_t id;
};


/**
 * This represent an exportable resource.
 * This is almost a copy of `struct resource` from the kernel
 * header `linux/ioport.h`.
 */
struct ddm_resource_user {
	struct ddm_resource_id id;
	struct ddm_resource_id parent;
	uint64_t start;
	uint64_t end;
	unsigned long flags;
};

#define IORESOURCE_TYPE_BITS	0x00001f00	/* Resource type */
#define IORESOURCE_IO		0x00000100
#define IORESOURCE_MEM		0x00000200
#define IORESOURCE_IRQ		0x00000400
#define IORESOURCE_DMA		0x00000800
#define IORESOURCE_BUS		0x00001000

struct ddm_device_user {
	char name[DDM_REGISTRATION_NAME_LEN];
	uint32_t id;

	struct ddm_resource_user res[DDM_REGISTRATION_RES_MAX_LEN];

	uint32_t priv[DDM_REGISTRATION_PRIV_MAX_LEN];
};

#endif
