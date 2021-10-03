/*
 * Copyright (c) 2020 Joep Buruma
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HAWKBIT_DEVICE_INFO_H__
#define __HAWKBIT_DEVICE_INFO_H__

#include <data/json.h>

#ifndef CONFIG_HAWKBIT_CUSTOM_DEVICE_INFO
/* Default cfg_data */
struct hawkbit_cfg_data {
	const char *VIN;
	const char *hwRevision;
};
#endif;

#ifdef CONFIG_HAWKBIT_CUSTOM_DEVICE_INFO
static const struct json_obj_descr json_cfg_data_descr[];
#else
static const struct json_obj_descr json_cfg_data_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct hawkbit_cfg_data, VIN, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hawkbit_cfg_data, hwRevision,
			    JSON_TOK_STRING),
};
#endif

/** @brief fill data whith the correct device information.
 * 
 */
void hawkbit_device_get_cfg_data(struct hawkbit_cfg_data *data);

#endif /* __HAWKBIT_DEVICE_INFO_H__