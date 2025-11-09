#include <Arduino.h>
#include <stdint.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"


// ---------- Sensor pins  ----------
const int light_sensor = A0;
const int IR_sensor = 12;

// ---------- States and Variables ----------
// Robot state
enum RobotState {
  idle = 0,
  moving,
  avoid_obstacle_left,
  avoid_obstacle_right,
  searching_puck,
  get_puck,
  finding_goal
};

RobotState robot__current_state = idle;

// ---------- lighting puck  ----------
// lighting threshold
// use tuning_ambient_light function to modify value for usage
int ambient_light = 800;
const int light_threshold = 20;

// find lighting obj flag
bool obj_found = false;
bool has_puck = false;

// sample infrared 
unsigned long start_time = 0;
const unsigned long sample_time = 100000;
int count_low = 0;
int total_count = 0;

// ---------- goal beacon  ----------
enum BeaconType{
  None = 0,
  B1500, 
  B600,
  Unclear
};

BeaconType searched_beacon = None;
BeaconType target_beacon = B1500;

// ---------- Functions ----------
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

void search_puck()
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
  move_forward_cl();
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
  delay(1000);
  stop_motors();
  delay(200);
  int right_reading = read_ambient_light(10);

  if (right_reading < left_reading)
    return false;
  
  rotate_ccw();
  delay(1000);

  return left_reading < right_reading ? true : false;
}

BeaconType detect_beacon(int detect_times = 3)
{
  int b600_count = 0, b1500_count = 0;

  for (int i = 0; i < detect_times; i++) {
    count_low = 0;
    total_count = 0;

    unsigned long t_end = micros() + sample_time;
    while ((long)(t_end - micros()) > 0) {   
      if (digitalRead(IR_sensor) == LOW)
        count_low++;
      total_count++;
    }

    if (total_count == 0) continue;

    float ratio = (float)count_low / (float)total_count;

    if (ratio > 0.26 && ratio < 0.32) {
      b600_count++;
    } else if (ratio > 0.17 && ratio < 0.22) {
      b1500_count++;
    }

    delay(10); 
  }

  if (b1500_count > b600_count && b1500_count > 0)
    return B1500;
  else if (b600_count > b1500_count && b600_count > 0)
    return B600;
  else if (b1500_count == 0 && b600_count == 0)
    return None;
  else
    return Unclear;
}


void search_goal()
{
  BeaconType bc;

  for (int i = 0; i < 20; i++){
    rotate_ccw();
    delay(200);
    stop_motors();
 
    bc = detect_beacon(10);
    if (bc == target_beacon){
      move_forward_cl();
      delay(1000);

      robot__current_state = finding_goal;
      break;
    }  
  }
}


// ---------- Initialization ----------
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

void update_robot_state()
{
  noInterrupts();

  bool right_touched = right_hit;
  bool left_touched = left_hit;
  bool target_touched = end_hit;

  // reset flag
  right_hit = left_hit = end_hit = false; 
  interrupts();
  
  // obstacle avoidance has highest priority
  if (right_touched){
    robot__current_state = avoid_obstacle_right;
    return;
  } 
  
  if (left_touched){
    robot__current_state = avoid_obstacle_left;
    return;
  } 

  if (target_touched){
    robot__current_state = get_puck;
    has_puck = true;
    return;
  }else if (!target_touched){
    has_puck = false;
  }

  obj_found = find_lighting_obj(read_ambient_light());
  if (obj_found){
    robot__current_state = searching_puck;
    return;
  }
  
  if (has_puck){
    searched_beacon = detect_beacon();
    if (searched_beacon != None && target_touched){
      robot__current_state = finding_goal;
      return;
    }
  }
  
  // no sensor is triggered => keep exploring
  robot__current_state = moving;  
}

// ---------- Main loop ----------
void loop() 
{
  // ======= test section =======
  
  // ============================

  // update Robot State
  update_robot_state();

  // robot behave with different state
  switch (robot__current_state){
    case(avoid_obstacle_right):
      move_backward_cl();
      delay(2000);
      turn_left_cl();
      delay(500);
      move_forward_cl();
      delay(1000);
      break;

    case(avoid_obstacle_left):
      move_backward_cl();
      delay(2000);
      turn_right_cl();
      delay(500);
      move_forward_cl();
      delay(1000);
      break;

    case(searching_puck):
      search_puck();
      break;

    case(get_puck):
      search_goal();
      break;

    case(finding_goal):
    case(moving):
      move_forward_cl();
      delay(1000);
      break;
    
    default:
      break;
  }
  
  Serial.print("robot state : ");
  Serial.println(robot__current_state);

  delay(50);
}
