#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <Arduino.h>

#define PUMP_AFTERRUN_TIME 300000 // 5 minutes in milliseconds

// Global relay state (Shadow Register)
// Bit 0 = Relay 1, Bit 1 = Relay 2, etc.
extern uint8_t relayShadow; 
// Manual override bitmask for relays 1-4 (1 = Manual, 0 = Auto)
extern uint8_t manualOverride;

/**
 * Updates the state of a specific relay channel (1-4)
 * This is the internal hardware setter.
 * @param channel The relay number (1, 2, 3, or 4)
 * @param state   True for ON (closed), False for OFF (open)
 */
void setRelay(uint8_t channel, bool state);

/**
 * Manually overrides a relay and sets its state.
 * Disables automation for this specific channel.
 */
void setRelayManual(uint8_t channel, bool state);

/**
 * Releases a relay channel back to automatic control.
 */
void releaseToAuto(uint8_t channel);

/**
 * Performs the background automation logic for all 4 relays.
 * Should be called in the slow loop (e.g., every 500ms).
 */
void updateRelayAutomation();

/**
 * Sends the current relayShadow state over the CAN bus.
 */
void sendRelayCommand();

#endif