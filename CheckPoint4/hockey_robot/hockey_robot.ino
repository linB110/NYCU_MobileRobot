#include <Arduino.h>
#include <stdint.h>
#include <MsTimer2.h>
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
  approaching_puck,
  get_puck,
  finding_goal
};

volatile RobotState robot_current_state = idle;

// Use timer interrupt to activate motion close loop control
const unsigned long Motion_control_period = 20;

// ---------- lighting puck  ----------
// lighting threshold
// use tuning_ambient_light function to modify value for usage
int ambient_light = 900;
const int light_threshold = 20;
const int close_to_puck = 100;
const int in_right_direction_threshold = 15;
int last_detected_light = 1024;

// find lighting obj (puck) flags
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
void motion_control_ISR()
{
  long r = read_right_pulse();
  long l = read_left_pulse();
  long diff = r - l;

  int pwmR = 0, pwmL = 0;
  const int max_comp = 60;     
  const int min_pwm  = 80;     
  const float Kp = 0.5f;
  const int ticks_tolerance = 50;

  long error = 0;
  const int turn_target = 300;

  auto apply_pid = [&](long error){
    if (abs(error) <= ticks_tolerance) error = 0;           
    int comp = (int)(Kp * error);
    comp = constrain(comp, -max_comp, max_comp);
    int base = min_pwm;
    int left  = max(base + comp, base);
    int right = max(base - comp, base);
    pwmL = left; 
    pwmR = right;

    motor_control(pwmR, IN1, IN2, ENA);
    motor_control(pwmL, IN3, IN4, ENB);
  };

  switch (robot_current_state){
    case (moving):
    case (approaching_puck):
      error = diff;                
      apply_pid(error);
      break;
    
    case(avoid_obstacle_left):
      error = diff - turn_target;  
      if (abs(error) <= ticks_tolerance) { robot_current_state = moving; break; }
      apply_pid(error);
      break;

    case(avoid_obstacle_right):
      error = diff - turn_target;  
      if (abs(error) <= ticks_tolerance) { robot_current_state = moving; break; }
      apply_pid(error);
      break;
  }
}
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

// ========================
void rotate_cw_step_ticks(long step_ticks)
 {
  long start = get_diff_ticks();

  rotate_cw();
  while (labs(get_diff_ticks() - start) < step_ticks) {
  }
  stop_motors();
}

void rotate_ccw_step_ticks(long step_ticks)
{
  if (step_ticks < 0) step_ticks = -step_ticks;

  long start = get_diff_ticks();

  rotate_ccw();
  while (labs(get_diff_ticks() - start) < step_ticks) {}
  stop_motors();
}

void rotate_ccw_to_diff(long target_diff)
{
  rotate_ccw();
  if (get_diff_ticks() > target_diff) {
    while (get_diff_ticks() > target_diff) {}
  } else {
    while (get_diff_ticks() < target_diff) {
    }
  }
  stop_motors();
}

void search_puck()
{
  const int   num_steps  = 7;     
  const long  step_ticks = 200;   

  int  best_light = 1024;
  int  best_step  = 0;

  stop_motors();
  delay(50);

  reset_ticks();
  long base_diff = get_diff_ticks();   

  for (int i = 0; i < num_steps; i++) {

    stop_motors();
    delay(50);                    

    int v = read_ambient_light(10);
    if (v < best_light) {
      best_light = v;
      best_step  = i;
    }

    rotate_cw_step_ticks(step_ticks);
  }

  long final_diff = get_diff_ticks();

  long target_diff = base_diff + (long)best_step * step_ticks;

  rotate_ccw_to_diff(target_diff);

  stop_motors();
  reset_ticks();
}

// ========================

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

    // checking infomation 
    // Serial.print("ratio = ");
    // Serial.println(ratio);

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


void searching_goal()
{
  BeaconType bc;

  for (int i = 0; i < 5; i++){
    rotate_ccw();
    delay(200);
    stop_motors();
 
    bc = detect_beacon(10);
    if (bc == target_beacon){
      move_forward_cl();
      delay(1000);

      robot_current_state = finding_goal;
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
  
  // compute motion parameter periodically
  MsTimer2::set(Motion_control_period, motion_control_ISR);
  MsTimer2::start();
 
  // ambient light calibration   
  ambient_light = read_ambient_light(15);  

  //robot_current_state = moving;
}

void update_robot_state()
{
  if (has_puck){
    robot_current_state = finding_goal;
    return;
  }
  
  int v = read_ambient_light();
  int diff = ambient_light - v;

  // direction is right => move forward
  if (diff > close_to_puck) {
    robot_current_state = approaching_puck;   
    return;
  }

  // direction could be wrong => search to make decision
  if (diff > light_threshold) {
      robot_current_state = searching_puck;;
    return;
  }
  
  if (has_puck){
    searched_beacon = detect_beacon();
    if (searched_beacon != None){
      robot_current_state = finding_goal;
      stop_motors();
      return;
    }
  }
  
  // no sensor is triggered => keep exploring
  robot_current_state = moving;  
}

void main_procedure()
{
  // update Robot State (if no obstacle to avoid)
  if (robot_current_state != avoid_obstacle_right &&
    robot_current_state != avoid_obstacle_left  &&
    robot_current_state != get_puck) {
    update_robot_state();
  }

  // robot behave with different state
  switch (robot_current_state){
    case(avoid_obstacle_right):
      MsTimer2::stop();

    {
      unsigned long t0 = millis();
      while (millis() - t0 < 2000) {
        move_backward_cl();
        delay(20); 
      }
    }

    {
      unsigned long t1 = millis();
      while (millis() - t1 < 500) {
        turn_left_cl();
        rotate_ccw_step_ticks(500);
        delay(20);
      }
    }

    MsTimer2::start();
    robot_current_state = moving;
    break;

      case(avoid_obstacle_left):
        MsTimer2::stop();

    {
      unsigned long t0 = millis();
      while (millis() - t0 < 2000) {
        move_backward_cl();
        delay(20);
      }
    }

    {
      unsigned long t1 = millis();
      while (millis() - t1 < 500) {
        rotate_cw_step_ticks(-500);
        turn_right_cl();
        delay(20);
      }
    }

    MsTimer2::start();
    robot_current_state = moving;
    break;

    case(searching_puck):
      MsTimer2::stop();
      search_puck();
      MsTimer2::start();
      robot_current_state = approaching_puck;
      break;

    case(get_puck):
      searching_goal();
      break;

    case (approaching_puck):
    case(finding_goal):
    case(moving):
      move_forward_cl();
      break;
    
    default:
      break;
  }
}

// ---------- Main loop ----------
void loop() 
{
  // ======= test section =======
  //detect_beacon();

  // int v = read_ambient_light();
  // Serial.print("lightness = ");
  // Serial.println(v);
  // delay(500);
  // turn_right_cl();
  //turn_left_cl();
  //move_forward_cl();
  // move_backward_cl();
  //motor_control(90, IN3, IN4, ENA);
  //digitalWrite(7, HIGH);
  //digitalWrite(8, LOW);
  //delay(1000);
  // stop_motors();
  // delay(1000);
  // int v = digitalRead(IN2);
  // Serial.print("lightness = ");
  // Serial.println(v);
  // ============================
   
  // obstacle avoidance has highest priority in all states
  noInterrupts();

  bool right_touched = right_hit;
  bool left_touched = left_hit;
  bool target_touched = end_hit;

  // reset flags
  right_hit = left_hit = end_hit = false; 

  interrupts();
      
  if (right_touched)
    robot_current_state = avoid_obstacle_right;
  
  if (left_touched)
    robot_current_state = avoid_obstacle_left;

  if (target_touched && !has_puck){
    robot_current_state = get_puck;
    has_puck = true;
  }//else
    //has_puck = false;

  main_procedure(); 
  
  delay(20);
}
