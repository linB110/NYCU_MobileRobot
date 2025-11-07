#include <Arduino.h>
#include <stdint.h>
#include <IRremote.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"

#define MOTOR_STEP 30
#define MOTOR_INCREMENT 5

// ---------- Sensor pins ----------
const int light_sensor = A0;
const int IR_sensor = 12;

// lighting threshold
// use tuning_ambient_light function to modify value for usage
int ambient_light = 800;
const int light_threshold = 20;

// find lighting obj flag
volatile bool obj_found = false;

// stop search
volatile bool capture_puck = false;

// sample infrared 
unsigned long start_time = 0;
const unsigned long sample_time = 100000;
int count_low = 0;
int total_count = 0;

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

void search()
{
  bool direction_is_left = find_obj_direction();
  if (direction_is_left) {
    rotate_ccw();
    delay(500);
  }else {
    rotate_cw();
    delay(500);
  }

  delay(100);
  move_forward();
  delay(300);
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
  delay(1500);
  stop_motors();
  delay(200);
  int right_reading = read_ambient_light(10);

  if (right_reading < left_reading)
    return false;
  
  rotate_ccw();
  delay(1000);

  return left_reading < right_reading ? true : false;
}

void setup() 
{
  Serial.begin(19200);
  
  // motor
  motor_encoder_begin();

  // snesor
  pinMode(light_sensor, INPUT);
  pinMode(IR_sensor, INPUT);
  sensor_init();      

  // ambient light calibration   
  ambient_light = read_ambient_light(15);  
}

void loop() 
{
  // ======= test section =======
  // int val = read_ambient_light();
  // Serial.println(val);

  // int lp = read_left_pulse();
  // Serial.print("left pulse = ");
  // Serial.println(lp);

  // int rp = read_right_pulse();
  // Serial.print("right pulse = ");
  // Serial.println(rp);

  count_low = 0;
  total_count = 0;

  unsigned long t_end = micros() + sample_time;
  while ((long)(t_end - micros()) > 0) {   
    int val = digitalRead(IR_sensor);
    total_count++;
    if (val == LOW) {
      count_low++;
    }
  }

  if (total_count == 0) {                
    Serial.println("total_count == 0");
  } else {
    float ratio = (float)count_low / (float)total_count;  
    Serial.print("ratio = ");
    Serial.println(ratio, 3);

    if (ratio > 0.26 && ratio < 0.32) {
      Serial.println("→ Beacon-1 (600)");
    } else if (ratio > 0.17 && ratio < 0.22) {
      Serial.println("→ Beacon-2 (1500)");
      move_forward();
      delay(1000);
      stop_motors();
    } else {
      Serial.println("→ Unknown / No beacon");
    }
  }

  //Serial.println(ratio);
  //Serial.println(ratio);
  // ============================


  //obstacle avoidance reading
  // noInterrupts();

  // bool right_touched = right_hit;
  // bool left_touched = left_hit;
  // bool target_touched = end_hit;
  // right_hit = left_hit = end_hit = false;

  // obj_found = find_lighting_obj(read_ambient_light());

  // interrupts();

  // //motion flow
  // if (!capture_puck){
  //   if (obj_found){
  //     search();
  //   }else{
  //     if (right_touched){
  //       move_backward_cl();
  //       delay(2000);
  //       search();
  //       //turn_left();
  //       //delay(1200);
  //     }else if (left_touched){
  //       move_backward_cl();
  //       delay(2000);
  //       search();
  //       //turn_right();
  //       //delay(1000);
  //     }else if (target_touched){
  //       capture_puck = true;
  //       stop_motors();
  //     }else{
  //       move_forward_cl();
  //     }
  //   }
  // }else{
  //     stop_motors();
  //   }
  
  // //move_forward_cl();
  delay(50);
}
