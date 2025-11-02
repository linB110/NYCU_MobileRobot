#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"

// ---------- Sensor pins ----------
const int touch_sensor_left = A1;
const int touch_sensor_right = A2;
const int touch_sensor_endmove = A3;

// touched flags
extern volatile bool right_hit;
extern volatile bool left_hit;
extern volatile bool end_hit;

// initialization
void sensor_init();

// obstacle avoidance
void touched_right();
void touched_left();
void touched_target();
