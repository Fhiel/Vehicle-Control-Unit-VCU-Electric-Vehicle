/**
 * @file utils.h
 * @brief Utility functions for checksums, logging, and thread-safe printing.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <cstdarg>
#include <cstddef>
#include <algorithm>

/**
 * @brief Simple XOR or Additive Checksum for RS485 packets.
 * @param data Pointer to the payload
 * @param len Length of the payload
 * @return uint8_t Calculated checksum
 */
uint8_t calculateChecksum(const uint8_t* data, size_t len);

/**
 * @brief Re-maps a number from one range to another (float version).
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Hex-dumps an RS485 packet to the serial console for debugging.
 */
void log_rs485_packet(const uint8_t* packet);

/**
 * @brief Thread-safe printf wrapper using a semaphore to prevent serial corruption.
 */
void safe_printf(const char* format, ...);

#endif // UTILS_H