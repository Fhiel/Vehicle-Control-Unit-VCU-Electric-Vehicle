/**
 * @file lock_control.cpp
 * @brief Automotive-grade locking logic for Type 2 charging connector with safe stop sequencing.
 * @author Fhiel (X1/9e Project)
 * @license MIT
 */

#define DEBUG
#include "lock_control.h"
#include "main.h"

// Declaration of your external CAN function (from CAN_Transmit.cpp)
extern void sendElconCommand(bool stopCharging);

/* --- Internal State Variables --- */
static LockState state = LOCK_IDLE;
static unsigned long actionStartTime = 0;
static unsigned long lastBmsStatus = 0x01;


/**
 * @brief Controls the DRV8871 H-Bridge for the Type 2 locking motor.
 * Updates the telemetry variables simultaneously.
 */
void setMotor(MotorState motorState) {
    switch (motorState) {
        case MOTOR_LOCK:
            digitalWrite(TYPE2_LOCK_IN1_PIN, HIGH);
            digitalWrite(TYPE2_LOCK_IN2_PIN, LOW);
            WITH_DATA_MUTEX({ telemetryData.lockIn1 = true; telemetryData.lockIn2 = false; });
            break;
        case MOTOR_UNLOCK:
            digitalWrite(TYPE2_LOCK_IN1_PIN, LOW);
            digitalWrite(TYPE2_LOCK_IN2_PIN, HIGH);
            WITH_DATA_MUTEX({ telemetryData.lockIn1 = false; telemetryData.lockIn2 = true; });
            break;
        case MOTOR_OFF:
        default:
            digitalWrite(TYPE2_LOCK_IN1_PIN, LOW);
            digitalWrite(TYPE2_LOCK_IN2_PIN, LOW);
            WITH_DATA_MUTEX({ telemetryData.lockIn1 = false; telemetryData.lockIn2 = false; });
            break;
    }
}

/**
 * @brief Polls the physical manual unlock button and updates telemetry.
 */
void updateManualUnlock() {
    static bool lastButtonState = false;
    bool currentButtonState = (digitalRead(TYPE2_MANUAL_UNLOCK_PIN) == LOW);
    
    // Write the current button state and feedback sensor to telemetry for 
    // dashboard display and CAN logic, ensuring thread safety with the mutex.
    WITH_DATA_MUTEX({
        telemetryData.manualUnlockBtn = currentButtonState;
        telemetryData.lockFeedback = (digitalRead(TYPE2_FEEDBACK_PIN) == LOW);
    });
    
    if (currentButtonState && !lastButtonState) {
        manualUnlockPressed = true; // Setzt den globalen Trigger
    }
    lastButtonState = currentButtonState;
}

/**
 * @brief Main state machine for the charging connector lock.
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
        plugDetected = (currentBmsStatus == BMS_STATUS_CHARGE || telemetryData.isCharging);
        unlockRequested = manualUnlockPressed; // True if physical button OR web-lightning clicked
        vifcStatus = telemetryData.vifcStatus;
        xSemaphoreGive(dataMutex);
    } else return;

    // 2. BMS Transition Watchdog (IMD Self-Test)
    if (currentBmsStatus != lastBmsStatus) {
        if (currentBmsStatus == BMS_STATUS_READY || currentBmsStatus == BMS_STATUS_CHARGE) {
            bool selfTestNeeded = (vifcStatus & (1 << 12)) || (vifcStatus & (1 << 13));
            if (selfTestNeeded && !telemetryData.selfTestRunning) {
                WITH_DATA_MUTEX({ telemetryData.selfTestRequested = true; });
                safe_printf("[LOCK] BMS Change: Triggering IMD Self-Test.\n");
            }
        }
        lastBmsStatus = currentBmsStatus;
    }

    // 3. Locking & Charging Sequence State Machine
    switch (state) {
        case LOCK_IDLE:
            // AUTO-LOCK LOGIC: Plug inserted -> Lock immediately and allow charge
            if (plugDetected && !telemetryData.isLocked) {
                state = LOCK_LOCKING;
                actionStartTime = now;
                setMotor(MOTOR_LOCK);
                WITH_DATA_MUTEX({ telemetryData.isLocking = true; });
                safe_printf("[LOCK] Plug detected. Engaging Auto-Lock.\n");
            } 
            // Manual Unlock while idle (e.g., if cable is locked but no charging session is running)
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
                bool feedbackOk = (digitalRead(TYPE2_FEEDBACK_PIN) == LOW);
                setMotor(MOTOR_OFF);
                
                state = feedbackOk ? LOCK_LOCKED : LOCK_IDLE;
                WITH_DATA_MUTEX({ 
                    telemetryData.isLocked = feedbackOk; 
                    telemetryData.isLocking = false;
                });

                if (feedbackOk) {
                    safe_printf("[LOCK] Connector LOCKED & SECURED. Charging allowed.\n");
                } else {
                    safe_printf("[LOCK] ERROR: Feedback failed. Bolt not engaged.\n");
                }
            }
            break;

        case LOCK_LOCKED:      
            // UNLOCK REQUEST (Lightning Bolt oder Physischer Button)
            if (unlockRequested) {
                safe_printf("[LOCK] Manual stop requested. Informing CAN Task...\n");
                WITH_DATA_MUTEX({ telemetryData.manualStopRequested = true; });
                
                state = LOCK_STOPPING_CHARGE;
                actionStartTime = now;
            }
            break;

        case LOCK_STOPPING_CHARGE:
            // We wait a short moment to ensure the CAN task has time to send the stop command to the charger before we cut power to the motor
            if (now - actionStartTime >= 500) { 
                safe_printf("[LOCK] Safety delay complete. Releasing locking pin...\n");
                
                state = LOCK_UNLOCKING;
                actionStartTime = now;
                setMotor(MOTOR_UNLOCK);
                
                WITH_DATA_MUTEX({ 
                    telemetryData.isUnLocking = true;
                    telemetryData.isLocked = false;
                    telemetryData.manualStopRequested = false; // Set back to false after informing CAN task
                    manualUnlockPressed = false; 
                });
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
            if (!unlockRequested) {
                state = LOCK_IDLE;
            }
            break;
    }
}