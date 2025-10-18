#pragma once
#include <Arduino.h>
#include <stdint.h>

// ---------- Motor pins ----------
extern const int ENA; // left wheel (PWM)
extern const int IN1;
extern const int IN2;
extern const int ENB; // right wheel (PWM)
extern const int IN3;
extern const int IN4;

// ---------- Encoder pins ----------
extern const int ENCODER_R_A;
extern const int ENCODER_R_B;
extern const int ENCODER_L_A;
extern const int ENCODER_L_B;

// ---------- Params ----------
#ifndef PPR
#define PPR 960.0f
#endif
#ifndef SPEED_WINDOW_MS
#define SPEED_WINDOW_MS 500
#endif

// initialization
void motor_encoder_begin();

// read RPM
float read_left_rpm();
float read_right_rpm();

// motor control
void motor_control(int pwm, int inA, int inB, int enable);

void move_forward(int pwm = 90);
void move_backward(int pwm = 90);
void turn_left(int inner_pwm = 90, int outer_pwm = 110);
void turn_right(int inner_pwm = 90, int outer_pwm = 110);
void stop_motors();

