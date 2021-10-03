/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hawkbit_device.h"
#include "hawkbit_device_info.h"
#include <string.h>

bool hawkbit_get_device_identity(char *id, int id_max_len)
{
	uint8_t hwinfo_id[DEVICE_ID_BIN_MAX_SIZE];
	ssize_t length;

	length = hwinfo_get_device_id(hwinfo_id, DEVICE_ID_BIN_MAX_SIZE);
	if (length <= 0) {
		return false;
	}

	memset(id, 0, id_max_len);
	length = bin2hex(hwinfo_id, (size_t)length, id, id_max_len - 1);

	return length > 0;
}

void __weak hawkbit_device_get_cfg_data(struct hawkbit_cfg_data *data)
{
	char device_id[DEVICE_ID_HEX_MAX_SIZE];
	if(!hawkbit_get_device_identity(device_id, DEVICE_ID_HEX_MAX_SIZE))
	{
		return;
	}
	memcpy(data->VIN, device_id, strlen(device_id));
	data->hwRevision = "3";
}