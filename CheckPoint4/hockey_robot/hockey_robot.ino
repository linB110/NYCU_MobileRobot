#include <Arduino.h>
#include <stdint.h>
#include <MsTimer2.h>
#include "motor_motion.h"
#include "obstacle_avoidance.h"
#include "searching.h"

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
RobotState prev_robot_state = idle;

// Use timer interrupt to activate motion close loop control
const unsigned long Motion_control_period = 20;

// ---------- lighting puck  ----------
// lighting threshold
// use tuning_ambient_light function to modify value for usage
const int close_to_puck = 50;
const int in_right_direction_threshold = 15;
int last_detected_light = 1024;

// find lighting obj (puck) flags
bool obj_found = false;
bool has_puck = false;
bool direction_locked = false;

// ---- puck tracking ----
const int num_good = 5;   
const int num_bad  = 5;   
const int light_eps = 5;  

int good_count = 0;
int bad_count  = 0;
int last_light = 1024;    

// +1 => cw, -1 => ccw
int heading_dir = 1;

// ---- anti-stuck ----
int bump_count = 0;              
const int bump_threshold = 3;    
bool need_full_search = false; 

// sample infrared 
unsigned long start_time = 0;
const unsigned long sample_time = 100000;
int count_low = 0;
int total_count = 0;

// robot got kinapped 
unsigned long last_bump_time = 0;
const unsigned long bump_time_window = 10000;

BeaconType searched_beacon = None;
BeaconType target_beacon = B600; // change it if different beacon is set

// ---------- Functions ----------
void register_bump()
{
  unsigned long now = millis();

  if (now - last_bump_time > bump_time_window) {
    bump_count = 0;
  }

  bump_count++;
  last_bump_time = now;
}

void motion_control_ISR()
{
  long r = read_right_pulse();
  long l = read_left_pulse();
  long diff = r - l;

  int pwmR = 0, pwmL = 0;
  const int max_comp = 80;     
  const int min_pwm  = 120;     
  const float Kp = 0.5f;
  const int ticks_tolerance = 50;

  long error = 0;
  const int turn_target = 300;

  auto apply_pid = [&](long error){
    if (abs(error) <= ticks_tolerance) error = 0;           
    int comp = (int)(Kp * error);
    comp = constrain(comp, -max_comp, max_comp);
    int base = min_pwm;
    int left  = base + comp;
    int right = base - comp;

    left  = constrain(left,  0, 255);
    right = constrain(right, 0, 255);

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

// ---------- Initialization ----------
void setup() 
{
  Serial.begin(19200);
  
  // motor
  motor_encoder_begin();

  // snesor
  touch_sensor_init(); 
  search_sensor_init();
  
  // compute motion parameter periodically
  MsTimer2::set(Motion_control_period, motion_control_ISR);
  MsTimer2::start();
 
  robot_current_state = moving;
}

void update_robot_state()
{
  if (has_puck){
    robot_current_state = finding_goal;
    return;
  }

  int v = read_light_sensor(5);
  int diff = ambient_light - v;

  // direction is right => move forward
  if (diff > close_to_puck) {
    robot_current_state = approaching_puck;
    direction_locked = true;   
    return;
  }

  
  // no sensor is triggered => keep exploring
  robot_current_state = moving;  
}

void main_procedure()
{
  // robot behave with different state
  switch (robot_current_state){
    case(avoid_obstacle_right):
      MsTimer2::stop();

    {
      unsigned long t0 = millis();
      while (millis() - t0 < 2000) {
        move_backward();
        delay(20); 
      }
    }

    {
      unsigned long t1 = millis();
      while (millis() - t1 < 500) {
        turn_left();
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
        move_backward();
        delay(20);
      }
    }

    {
      unsigned long t1 = millis();
      while (millis() - t1 < 500) {
        turn_right();
        delay(20);
      }
    }

    MsTimer2::start();
    robot_current_state = moving;
    break;

    case(searching_puck):
      MsTimer2::stop();
      if (try_search_puck()){
        robot_current_state = moving;
        MsTimer2::start();
        break;
      }else{
        robot_current_state = moving;
        MsTimer2::start();
        break;
      }

      MsTimer2::start();
      robot_current_state = approaching_puck;
      break;

    case(get_puck):
      //searching_goal();
      break;

    case(finding_goal):
    case(moving):
      if (prev_robot_state != moving) {
        reset_ticks();          
      }

      //move_forward();
      break;
    
    default:
      break;
  }
}

bool locked = false; 

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
  //delay(1000);
  // stop_motors();
  // delay(1000);
  // int v = analogRead(A0);
  // Serial.print("lightness = ");
  // Serial.println(v);
  // delay(1000);

  // int r = read_right_pulse();
  // Serial.print("pulse = ");
  // Serial.println(r);

  // rotate_angle(90.0);
  // delay(2000);

  // ============================

  // obstacle avoidance has highest priority in all states
  noInterrupts();

  bool right_touched = right_hit;
  bool left_touched = left_hit;
  bool target_touched = end_hit;

  // reset flags
  right_hit = left_hit = end_hit = false; 

  interrupts();
      
  if (right_touched) {
    // robot_current_state = avoid_obstacle_right;
    //move_backward();
    //delay(1000);
    need_full_search = true;
    register_bump();                      
  }
  
  if (left_touched) {
    // robot_current_state = avoid_obstacle_left;
    //move_backward();
    //delay(1000);
    need_full_search = true;
    register_bump();                     
  }

  if (target_touched && !has_puck){
    has_puck = true;
    MsTimer2::stop();
    if (try_search_goal()){
      robot_current_state = moving;
    }
    MsTimer2::start();
  }
 
  // lost the puck
  if (has_puck && read_light_sensor(5) > 300){
    has_puck = false;
  }

  if (bump_count >= bump_threshold) {
    need_full_search = true;
    bump_count = 0;                    
  }
  
  if (need_full_search) {
    MsTimer2::stop();
    move_backward();
    delay(800);
    stop_motors();
    MsTimer2::start();
    robot_current_state = searching_puck;
    need_full_search = false;
  }
  
  if (robot_current_state == moving || robot_current_state == idle) {
    update_robot_state();
  }

  prev_robot_state = robot_current_state;

  main_procedure(); 
  
  delay(10);
}
