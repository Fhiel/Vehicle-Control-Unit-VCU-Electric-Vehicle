// rs485.h
#ifndef RS485_H
#define RS485_H

#include <cstdint>
#include "main.h"
#include "utils.h"

#define PACKET_PAYLOAD_LENGTH 14
#define PACKET_TOTAL_LENGTH   17   // 14 + Start + Len + CRC

void packTelemetryData(uint8_t* payload_buffer);
void send_rs485_telemetry();

#endif