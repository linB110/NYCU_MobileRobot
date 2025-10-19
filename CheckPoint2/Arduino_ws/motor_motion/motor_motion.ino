#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"

// ---------- Sensor pins ----------
const int light_sensor = A0;

// lighting threshold
const int ambient_light;
const int lighting_obj;

void tuning_ambient_light()
{
  // light sensor : dark = 1,  bright = 0
  int val = analogRead(light_sensor);
  Serial.print("val = ");
  Serial.println(val);
}

void setup() 
{
  Serial.begin(9600);
  
  // motor
  motor_encoder_begin();

  // snesor
  pinMode(light_sensor, INPUT);
  sensor_init();           
}

void loop() 
{
  //int val = digitalRead(light_sensor);
  //Serial.print("val = ");

  // touch sensor : opened = 1 touched = 0
  //Serial.println(val);

  //move_forward();
  // turn_right();
  // delay(10000);
  // stop_motors();
  // delay(5000);
  // turn_left();
  // delay(10000);
  // stop_motors();
  
  // noInterrupts();
  // bool right_touched = right_hit;
  // bool left_touched = left_hit;
  // bool target_touched = end_hit;
  // right_hit = left_hit = end_hit = false;
  // interrupts();
  // Serial.println(right_touched);
  
  // if (right_touched){
  //   move_backward();
  //   delay(2000);
  //   turn_left();
  //   delay(1000);
  // }else if (left_touched){
  //   move_backward();
  //   delay(2000);
  //   turn_right();
  //   delay(1000);
  // }else if (target_touched){
  //   stop_motors();
  // }else{
  //   move_forward();
  // }
  
  //tuning_ambient_light();
  delay(10000);
}
