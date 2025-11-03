// can_process.h
#ifndef CAN_PROCESS_H
#define CAN_PROCESS_H

#include "main.h"
#include <cstdint>

void process_can_messages(void *parameter);
void twai_receive_task(void *parameter);
void twai_monitor_task(void *parameter);
void check_data_timeout(unsigned long currentMillis);
void handleQueueMonitoring(unsigned long currentMillis);
void handleCanBus();
void add_received_id(uint32_t id);

#endif