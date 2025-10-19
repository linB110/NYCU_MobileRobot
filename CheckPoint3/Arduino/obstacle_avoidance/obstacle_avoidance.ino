#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"

#define MOTOR_STEP 10
#define MOTOR_INCREMENT 5

// ---------- Sensor pins ----------
const int light_sensor = A0;

// lighting threshold
// use tuning_ambient_light function to modify value for usage
const int ambient_light = 540;
const int light_threshold = 150;

// find lighting obj flag
volatile bool obj_found = false;

int read_ambient_light()
{
  // light sensor : dark = 1,  bright = 0
  int val = analogRead(light_sensor);
  Serial.print("val = ");
  Serial.println(val);

  return val;
}

bool find_lighting_obj(int detected_light_value)
{
  if (detected_light_value <= ambient_light - light_threshold)
    return true;
  
  return false; 
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
  

  noInterrupts();

  bool right_touched = right_hit;
  bool left_touched = left_hit;
  bool target_touched = end_hit;
  right_hit = left_hit = end_hit = false;

  bool found = find_lighting_obj(read_ambient_light());

  interrupts();
  //Serial.println(right_touched);
  
  if (obj_found){
    for (int i = 0; i < MOTOR_STEP; i += MOTOR_INCREMENT){
      rotate();
      delay(500);
      
      if ( find_lighting_obj(read_ambient_light()) )
        break;
    }

    move_forward();
    delay(2000);
  }  
  else{
    if (right_touched){
      move_backward();
      delay(2000);
      turn_left();
      delay(1000);
    }else if (left_touched){
      move_backward();
      delay(2000);
      turn_right();
      delay(1000);
    }else if (target_touched){
      stop_motors();
    }else{
      move_forward();
    }
  }
  
  // Serial.println(found);

  // if (found)
  //   Serial.println("found");

  // delay(1000);
  //rotate();
  delay(1000);
}
