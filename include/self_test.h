// self_test.h
#ifndef SELF_TEST_H
#define SELF_TEST_H

#include "main.h"
#include <cstdint>

esp_err_t get_imd_hv_relays(uint8_t *neg_state, uint8_t *pos_state);
void send_imd_request(uint16_t cmd, uint8_t param);
void self_test_task(void *parameter);

#endif