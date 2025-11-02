#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"

// ---------- Sensor pins ----------
const int light_sensor = A0;
const int IR_sensor = 12;

void setup() 
{
  Serial.begin(9600);
  //pinMode(IR_sensor, INPUT);
  motor_encoder_begin();
}

void loop() 
{
  // ======= IR test ========
  // unsigned long durHigh = pulseIn(IR_sensor, HIGH, 200000); // timeout 200ms
  // unsigned long durLow  = pulseIn(IR_sensor, LOW, 200000);

  // if (durHigh == 0 && durLow == 0) {
  //   // no signal
  //   Serial.println("No pulse");
  //   delay(200);
  //   return;
  // }

  // Serial.print("High(us): "); Serial.print(durHigh);
  // Serial.print("  Low(us): "); Serial.println(durLow);
  // delay(1000);
  // ======================

  // ======= test ========
  //move_forward();
  //int lp = read_left_pulse();
  // Serial.print("left pulse = ");
  // Serial.println(lp);

  //int rp = read_right_pulse();
  // Serial.print("right pulse = ");
  // Serial.println(rp);
  
  //motor_control(80, IN1, IN2, ENA);
  //move_forward_cl();
  //move_backward_cl();




  delay(50);
  // ======================
}
