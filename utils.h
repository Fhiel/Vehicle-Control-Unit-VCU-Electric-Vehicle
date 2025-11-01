// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <cstdint>      // <--- HINZUFÜGEN!
#include <cstdarg>
#include <cstddef>
#include <algorithm>

uint8_t calculateChecksum(const uint8_t* data, size_t len);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void log_rs485_packet(const uint8_t* packet);
void safe_printf(const char* format, ...);

#endif

