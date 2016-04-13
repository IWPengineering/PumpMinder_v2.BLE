#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ble_display_service.h"
#include "pstorage.h"
#include <string.h>
#include "preprocessor.h"

#define PERM_STORAGE_BLOCK_SIZE		8
#define PERM_STORAGE_BLOCK_COUNT	257

void init_storage(void);
uint8_t get_next_record(void);
bool get_record(ble_display_service_record_t *p_record);
bool save_record(ble_display_service_record_t *p_record);
