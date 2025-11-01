// pump_control.h
#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include "main.h"
#include <cstdint>

void update_inv_pump(int8_t mcu_temp, bool valid);
void update_bat_pump(float bat_temp, bool valid);

#endif