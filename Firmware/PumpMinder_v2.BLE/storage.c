#include "storage.h"

static pstorage_handle_t pstorage_block_base_id = { .module_id = 0x1 };
static bool is_pstorage_busy = false;
static uint8_t cur_record_ptr = 0;

static void pstorage_cb(pstorage_handle_t *p_handle, 
	uint8_t op_code,
	uint32_t result,
	uint8_t *p_data,
	uint32_t data_len)
{
	switch (op_code)
	{
	case PSTORAGE_STORE_OP_CODE:
		break;
	case PSTORAGE_LOAD_OP_CODE:
		break;
	case PSTORAGE_CLEAR_OP_CODE:
		break;
	case PSTORAGE_UPDATE_OP_CODE:
		is_pstorage_busy = false;
		break;
	default:
		break;
	}
}

void init_storage(void)
{
	uint32_t err_code;
	
	pstorage_module_param_t param = 
	{
		.cb = pstorage_cb,
		.block_size = PERM_STORAGE_BLOCK_SIZE,
		.block_count = PERM_STORAGE_BLOCK_COUNT
	};
	
	err_code = pstorage_register(&param, &pstorage_block_base_id);
	if (err_code != NRF_SUCCESS)
	{
		// Handle error
		BREAK_OUT();
	}
}

uint8_t get_next_record(void)
{
	return cur_record_ptr;
}

static uint8_t dest_array[PERM_STORAGE_BLOCK_SIZE] __attribute__((aligned(4)));
bool get_record(ble_display_service_record_t *p_record)
{
	uint32_t err_code;
	
	pstorage_handle_t cur_block_id;
	
	err_code = pstorage_block_identifier_get(&pstorage_block_base_id,
		p_record->record_number,
		&cur_block_id);
	if (err_code != NRF_SUCCESS)
	{
		// Handle error
		BREAK_OUT();
	}
	
	err_code = pstorage_load(dest_array, &cur_block_id, PERM_STORAGE_BLOCK_SIZE, 0);
	if (err_code != NRF_SUCCESS)
	{
		// Handle error
		BREAK_OUT();
	}
	
	// The information should already be here - 
	//  pstorage load gets info before returning
	memcpy(&p_record->timestamp, dest_array, 4);
	memcpy(&p_record->seconds_running, dest_array + 4, 4);
	
	return true;
}

static uint8_t pD[PERM_STORAGE_BLOCK_SIZE] __attribute__((aligned(4)));
bool save_record(ble_display_service_record_t *p_record)
{
	if (is_pstorage_busy)
	{
		return false;
	}
	
	uint32_t err_code;
	
	pstorage_handle_t cur_block_id;
	
	err_code = pstorage_block_identifier_get(&pstorage_block_base_id, 
		p_record->record_number,
		&cur_block_id);
	if (err_code != NRF_SUCCESS)
	{
		// Handle error
		BREAK_OUT();
	}
	
	memcpy(pD, &p_record->timestamp, 4);
	memcpy(pD + 4, &p_record->seconds_running, 4);
	
	err_code = pstorage_update(&cur_block_id, pD, PERM_STORAGE_BLOCK_SIZE, 0);
	if (err_code != NRF_SUCCESS)
	{
		// Handle error
		BREAK_OUT();
	}
	
	is_pstorage_busy = true;
	
	if (cur_record_ptr == 0xFF)
	{
		cur_record_ptr = 0;
	}
	else
	{
		cur_record_ptr++;
	}
	
	return true;
}










