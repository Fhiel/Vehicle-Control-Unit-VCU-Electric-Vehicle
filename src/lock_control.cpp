/**
 * @file lock_control.cpp
 * @brief Automotive-grade locking logic for Type 2 charging connector.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 * * Logic Overview:
 * 1. Auto-Lock: As soon as BMS reports "CHARGE" status (Plug detected via PP), 
 * the VCU automatically engages the mechanical lock.
 * 2. Interlock: While locked or charging, the VCU signals 'Drive Inhibit' to the MCU.
 * 3. Manual Unlock: Only possible via button if charging is not active/stopped.
 */

#define DEBUG
#include "lock_control.h"
#include "main.h"

/* --- Internal State Variables --- */
static LockState state = LOCK_IDLE;
static unsigned long actionStartTime = 0;
static unsigned long lastBmsStatus = 0x01;

/**
 * @brief Controls the H-Bridge for the Type 2 locking motor.
 */
void setMotor(MotorState motorState) {
    switch (motorState) {
        case MOTOR_LOCK:
            digitalWrite(TYPE2_LOCK_PIN, HIGH);
            digitalWrite(TYPE2_UNLOCK_PIN, LOW);
            break;
        case MOTOR_UNLOCK:
            digitalWrite(TYPE2_LOCK_PIN, LOW);
            digitalWrite(TYPE2_UNLOCK_PIN, HIGH);
            break;
        case MOTOR_OFF:
        default:
            digitalWrite(TYPE2_LOCK_PIN, LOW);
            digitalWrite(TYPE2_UNLOCK_PIN, LOW);
            break;
    }
}

/**
 * @brief Polls the physical manual unlock button (Active LOW).
 */
void updateManualUnlock() {
    static bool lastButtonState = false;
    bool currentButtonState = (digitalRead(TYPE2_MANUAL_UNLOCK_PIN) == LOW);
    
    if (currentButtonState && !lastButtonState) {
        manualUnlockPressed = true;
    }
    lastButtonState = currentButtonState;
}

/**
 * @brief Main state machine for the charging connector lock.
 * Orchestrates Auto-Lock on plug detection and safety-checked unlocking.
 */
void handleLockState() {
    updateManualUnlock();
    unsigned long now = millis();

    // 1. Thread-safe Data Acquisition
    uint8_t currentBmsStatus = 0;
    bool plugDetected = false;
    bool unlockRequested = false;
    uint16_t vifcStatus = 0;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentBmsStatus = telemetryData.bmsStatus;
        // In your system, BMS "CHARGE" state (0x04) indicates plug is inserted (PP)
        plugDetected = (currentBmsStatus == 0x04 || telemetryData.is_charging);
        unlockRequested = manualUnlockPressed;
        vifcStatus = telemetryData.vifcStatus;
        xSemaphoreGive(dataMutex);
    } else return;

    // 2. BMS Transition Watchdog (Trigger Self-Test on startup/charge)
    if (currentBmsStatus != lastBmsStatus) {
        if (currentBmsStatus == 0x02 || currentBmsStatus == 0x04) {
            bool selfTestNeeded = (vifcStatus & (1 << 12)) || (vifcStatus & (1 << 13));
            if (selfTestNeeded && !telemetryData.selfTestRunning) {
                WITH_DATA_MUTEX({ telemetryData.selfTestRequested = true; });
                safe_printf("[LOCK] BMS Change: Triggering IMD Self-Test.\n");
            }
        }
        lastBmsStatus = currentBmsStatus;
    }

    // 3. Locking State Machine
    switch (state) {
        case LOCK_IDLE:
            // AUTO-LOCK LOGIC: Plug inserted? -> Lock immediately
            if (plugDetected && !telemetryData.isLocked) {
                state = LOCK_LOCKING;
                actionStartTime = now;
                setMotor(MOTOR_LOCK);
                WITH_DATA_MUTEX({ telemetryData.isLocking = true; });
                safe_printf("[LOCK] Plug detected (PP). Engaging Auto-Lock.\n");
            } 
            // Manual Unlock (only if no plug is present or specifically requested)
            else if (unlockRequested && !plugDetected) {
                state = LOCK_UNLOCKING;
                actionStartTime = now;
                setMotor(MOTOR_UNLOCK);
                WITH_DATA_MUTEX({ 
                    telemetryData.isUnLocking = true;
                    manualUnlockPressed = false; 
                });
            }
            break;

        case LOCK_LOCKING:
            if (now - actionStartTime >= LOCK_TIME_MS) {
                // Verify physical feedback from the lock bolt
                bool feedbackOk = (digitalRead(TYPE2_FEEDBACK_PIN) == LOW);
                setMotor(MOTOR_OFF);
                
                state = feedbackOk ? LOCK_LOCKED : LOCK_IDLE;
                WITH_DATA_MUTEX({ 
                    telemetryData.isLocked = feedbackOk; 
                    telemetryData.isLocking = false;
                });

                if (feedbackOk) safe_printf("[LOCK] Connector LOCKED & SECURED.\n");
                else safe_printf("[LOCK] ERROR: Feedback failed. Bolt not engaged.\n");
            }
            break;

        case LOCK_LOCKED:
            // The connector stays locked until the user explicitly pushes the button.
            // Even if charging finishes, the cable remains theft-protected.
            if (unlockRequested) {
                // SAFETY: Charging must be stopped before unlocking (checked in CAN_Transmit)
                state = LOCK_UNLOCKING;
                actionStartTime = now;
                setMotor(MOTOR_UNLOCK);
                WITH_DATA_MUTEX({ 
                    telemetryData.isUnLocking = true;
                    telemetryData.isLocked = false;
                    manualUnlockPressed = false;
                });
                safe_printf("[LOCK] User requested UNLOCK. Stopping motor...\n");
            }
            break;

        case LOCK_UNLOCKING:
            if (now - actionStartTime >= UNLOCK_TIME_MS) {
                state = LOCK_UNLOCKED;
                setMotor(MOTOR_OFF);
                WITH_DATA_MUTEX({ telemetryData.isUnLocking = false; });
                safe_printf("[LOCK] Motor OFF. Connector is now RELEASED.\n");
            }
            break;

        case LOCK_UNLOCKED:
            // Return to IDLE to allow new Auto-Lock cycles
            if (!unlockRequested) {
                state = LOCK_IDLE;
            }
            break;
    }
}