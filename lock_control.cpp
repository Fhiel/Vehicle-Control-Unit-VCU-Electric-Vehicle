// lock_control.cpp
#define DEBUG
#include "lock_control.h"
#include "main.h"  // für TelemetryData, dataMutex, etc.

// --- Externe Abhängigkeiten ---
extern SemaphoreHandle_t dataMutex;
extern bool dataMutexInitialized;
extern bool isLocked;
extern bool isUnlocking;
extern bool isLocking;
extern bool shouldLock;
extern bool is_charging;
extern bool manualUnlockPressed;
extern bool selfTestRunning;
extern bool selfTestRequested;
extern void safe_printf(const char *fmt, ...);

// --- Globale Variablen (statisch in .cpp) ---
static LockState state = LOCK_IDLE;
static unsigned long actionStartTime = 0;
static unsigned long lockFeedbackCheckTime = 0;
static const unsigned long lock_time = LOCK_TIME_MS;
static const unsigned long unlock_time = UNLOCK_TIME_MS;
static uint8_t last_bms_status = 0x01;
static unsigned long lastDebugPrint = 0;

// --- Motor-Steuerung ---
void setMotor(MotorState state) {
    switch (state) {
        case MOTOR_LOCK:
            digitalWrite(TYPE2_LOCK_PIN, HIGH);
            digitalWrite(TYPE2_UNLOCK_PIN, LOW);
            break;
        case MOTOR_UNLOCK:
            digitalWrite(TYPE2_LOCK_PIN, LOW);
            digitalWrite(TYPE2_UNLOCK_PIN, HIGH);
            break;
        case MOTOR_OFF:
            digitalWrite(TYPE2_LOCK_PIN, LOW);
            digitalWrite(TYPE2_UNLOCK_PIN, LOW);
            break;
    }
}

// --- Manuelle Entriegelung (Taster) ---
void updateManualUnlock() {
    static bool lastState = false;
    bool currentState = (digitalRead(TYPE2_MANUAL_UNLOCK_PIN) == LOW);  // Pull-up?
    if (currentState && !lastState) {
        manualUnlockPressed = true;
    }
    lastState = currentState;
}

// --- Hauptfunktion: Zustandsmaschine ---
void handleLockState() {
    updateManualUnlock();
    unsigned long currentMillis = millis();

    // --- 1. Lese TelemetryData ---
    bool mutex_taken = false;
    uint8_t temp_bms_status = 0;
    bool temp_is_charging = false;
    uint16_t temp_vifcStatus = 0;
    bool temp_manualUnlockPressed = false;

    if (dataMutexInitialized && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        mutex_taken = true;
        temp_bms_status = telemetryData.bmsStatusValid ? telemetryData.bmsStatus : 0;
        temp_is_charging = is_charging;
        temp_vifcStatus = telemetryData.vifcStatusValid ? telemetryData.vifcStatus : 0;
        temp_manualUnlockPressed = manualUnlockPressed;
    } else {
        safe_printf("handleLockState: Timeout dataMutex\n");
        return;
    }

    // --- 2. BMS-Übergänge → Selbsttest prüfen ---
    if (temp_bms_status != last_bms_status) {
        if ((last_bms_status == 0x01 && temp_bms_status == 0x02) ||
            (last_bms_status == 0x01 && temp_bms_status == 0x03) ||
            (last_bms_status == 0x02 && temp_bms_status == 0x03)) {
            bool selfTestOverall = (temp_vifcStatus & (1 << 12)) != 0;
            bool selfTestParamConfig = (temp_vifcStatus & (1 << 13)) != 0;
            if ((selfTestOverall || selfTestParamConfig) && !selfTestRunning && !selfTestRequested) {
                WITH_DATA_MUTEX({
                    selfTestRequested = true;
                });
                if (currentMillis - lastDebugPrint >= 5000) {
                    safe_printf("handleLockState: Selbsttest angefordert\n");
                    lastDebugPrint = currentMillis;
                }
            }
        }
        last_bms_status = temp_bms_status;
    }

    // --- 3. Zustandsmaschine ---
    switch (state) {
        case LOCK_IDLE:
            if (shouldLock && !isLocked && !temp_is_charging) {
                state = LOCK_LOCKING;
                WITH_DATA_MUTEX({ isLocking = true; });
                actionStartTime = currentMillis;
                lockFeedbackCheckTime = currentMillis + 1000;
                setMotor(MOTOR_LOCK);
            } else if (temp_manualUnlockPressed && !temp_is_charging) {
                state = LOCK_UNLOCKING;
                WITH_DATA_MUTEX({
                    isUnlocking = true;
                    isLocked = false;
                    manualUnlockPressed = false;
                });
                actionStartTime = currentMillis;
                setMotor(MOTOR_UNLOCK);
            }
            break;

        case LOCK_LOCKING:
            if (currentMillis - actionStartTime >= lock_time) {
                if (currentMillis >= lockFeedbackCheckTime) {
                    bool feedback = (digitalRead(TYPE2_FEEDBACK_PIN) == LOW);
                    if (feedback) {
                        state = LOCK_LOCKED;
                        WITH_DATA_MUTEX({
                            isLocking = false;
                            isLocked = true;
                        });
                        setMotor(MOTOR_OFF);
                    } else {
                        safe_printf("handleLockState: Verriegelung fehlgeschlagen\n");
                        state = LOCK_IDLE;
                        WITH_DATA_MUTEX({
                            isLocking = false;
                            isLocked = false;
                        });
                        setMotor(MOTOR_OFF);
                    }
                }
            }
            break;

        case LOCK_UNLOCKING:
            if (currentMillis - actionStartTime >= unlock_time) {
                state = LOCK_UNLOCKED;
                WITH_DATA_MUTEX({ isUnlocking = false; });
                setMotor(MOTOR_OFF);
            }
            break;

        case LOCK_LOCKED:
            if (temp_manualUnlockPressed && !temp_is_charging) {
                state = LOCK_UNLOCKING;
                WITH_DATA_MUTEX({
                    isUnlocking = true;
                    isLocked = false;
                    manualUnlockPressed = false;
                });
                actionStartTime = currentMillis;
                setMotor(MOTOR_UNLOCK);
            }
            break;

        case LOCK_UNLOCKED:
            if (shouldLock && !temp_is_charging) {
                state = LOCK_LOCKING;
                WITH_DATA_MUTEX({ isLocking = true; });
                actionStartTime = currentMillis;
                lockFeedbackCheckTime = currentMillis + 1000;
                setMotor(MOTOR_LOCK);
            }
            break;
    }

    // --- 4. Mutex freigeben ---
    if (mutex_taken) {
        xSemaphoreGive(dataMutex);
    }
}