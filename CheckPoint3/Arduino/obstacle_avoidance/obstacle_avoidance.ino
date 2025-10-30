#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"

#define MOTOR_STEP 30
#define MOTOR_INCREMENT 5

// ---------- Sensor pins ----------
const int light_sensor = A0;

// lighting threshold
// use tuning_ambient_light function to modify value for usage
int ambient_light = 800;
const int light_threshold = 20;

// find lighting obj 
volatile bool obj_found = false;
volatile int min_lightness_value = 1023;

// stop motion
volatile bool all_motion_stop = false;

int read_ambient_light(int samples = 5)
{
  int v = 0;
  for (int i = 0 ; i < samples; i++)
    v += analogRead(light_sensor);
  
  return v / samples;
}

bool find_lighting_obj(int detected_light_value)
{
  if (detected_light_value <= ambient_light - light_threshold)
    return true;
  
  return false; 
}

bool find_obj_direction()
{
  // rotate left and read
  rotate_ccw();
  delay(1000);
  stop_motors();
  delay(200);
  int left_reading = read_ambient_light(10);

  // rotate right and read
  rotate_cw();
  delay(2000);
  stop_motors();
  delay(200);
  int right_reading = read_ambient_light(10);

  rotate_ccw();
  delay(1000);

  return left_reading < right_reading ? true : false;
}

void setup() 
{
  Serial.begin(9600);
  
  // motor
  motor_encoder_begin();

  // snesor
  pinMode(light_sensor, INPUT);
  sensor_init();      

  // ambient light calibration   
  //ambient_light = read_ambient_light(15);  
}

void loop() 
{
  // int val = read_ambient_light();
  // Serial.println(val);

  //obstacle avoidance reading
  noInterrupts();

  bool right_touched = right_hit;
  bool left_touched = left_hit;
  bool target_touched = end_hit;
  right_hit = left_hit = end_hit = false;

  bool obj_found = find_lighting_obj(read_ambient_light());
  
  interrupts();
  
  int read_value = read_ambient_light();
  bool become_darker = true;
  if (read_value < min_lightness_value){
    min_lightness_value = read_value;
    become_darker = false;
  }
  
  // motion flow
  if (!all_motion_stop){
    if (obj_found){
      if (become_darker){
        bool direction_is_left = find_obj_direction();
        if (direction_is_left) {
          rotate_ccw();
        }else {
          rotate_cw();
        }
        delay(100);
        move_forward();
        delay(1000);
      }else{
        move_forward();
        delay(1000);
      }
    }else{
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
        all_motion_stop = true;
        stop_motors();
      }else{
        move_forward();
      }
    }
  }else{
      stop_motors();
    }
  
  delay(1000);
}
