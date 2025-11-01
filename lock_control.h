// lock_control.h
#ifndef LOCK_CONTROL_H
#define LOCK_CONTROL_H

// Zustandsmaschine
typedef enum {
    LOCK_IDLE,
    LOCK_LOCKING,
    LOCK_LOCKED,
    LOCK_UNLOCKING,
    LOCK_UNLOCKED
} LockState;

// Motor-Steuerung
typedef enum {
    MOTOR_OFF,
    MOTOR_LOCK,
    MOTOR_UNLOCK
} MotorState;

void handleLockState();
void setMotor(MotorState state);
void updateManualUnlock();

#endif