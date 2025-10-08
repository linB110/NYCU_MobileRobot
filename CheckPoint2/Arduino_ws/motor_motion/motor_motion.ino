#include <Arduino.h>
#include <stdint.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <ros_motion/pwmValue.h> // self-defined ros msg

// ---------- Motor pins ----------
const int ENA = 6; // left wheel (PWM)
const int IN1 = 10;
const int IN2 = 11;
const int ENB = 5; // right wheel (PWM)
const int IN3 = 8;
const int IN4 = 9;

// ---------- Encoder pins ----------
const int ENCODER_R_A = 2;   
const int ENCODER_R_B = 7;
const int ENCODER_L_A = 3;   
const int ENCODER_L_B = 13;

#define PPR 960.0f            
#define SPEED_WINDOW_MS 500   
#define PID_WINDOW_MS 50   
#define ROS_SPIN_TIME 5

volatile long right_ticks = 0;
volatile long left_ticks  = 0;

unsigned long last_speed_ts = 0;
unsigned long last_pid_ts = 0;
long last_right_ticks = 0;
long last_left_ticks  = 0;

struct PID_param
{
  float Kp, Ki, Kd;
  float pre_error;
  float integral;
};

PID_param pid_left{1.8, 0.3, 0.02, 0, 0};
PID_param pid_right{1.2, 0.25, 0.005, 0, 0};
float compute_pid(const float setpoint, float measurement, PID_param& pid_obj, float dt);

// ---------- Control ----------
float desire_RPM_L;
float desire_RPM_R;
float meas_left_rpm  = 0.0f;
float meas_right_rpm = 0.0f;

ros::NodeHandle node;
void motor_control(int pwm, int inA, int inB, int enable);
void motor_acuation(const ros_motion::pwmValue& inputValue);
ros::Subscriber<ros_motion::pwmValue> sub("motor_pwm", &motor_acuation);

void right_wheel_isr() 
{
  if (digitalRead(ENCODER_R_B) == HIGH) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}

void left_wheel_isr()
{
  if (digitalRead(ENCODER_L_B) == HIGH) {
    left_ticks--;
  } else {
    left_ticks++;
  }
}

void encoder_init() 
{
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  attachInterrupt(0, right_wheel_isr, RISING);
  attachInterrupt(1, left_wheel_isr, RISING);
}

void motor_control(int pwm, int inA, int inB, int enable) 
{
  bool isForward = (pwm >= 0);
  int pwmVal = abs(pwm);
  pwmVal = constrain(pwmVal, 0, 255);

  digitalWrite(inA, isForward ? HIGH : LOW);
  digitalWrite(inB, isForward ? LOW  : HIGH);
  analogWrite(enable, pwmVal);
}

void motor_acuation(const ros_motion::pwmValue& inputValue)
{
  //motor_control(inputValue.leftPWMValue,  IN1, IN2, ENA);
  //motor_control(inputValue.rightPWMValue, IN3, IN4, ENB);
  desire_RPM_L = inputValue.leftPWMValue;
  desire_RPM_R = inputValue.rightPWMValue;
}

float compute_pid(const float setpoint, float measurement, PID_param& pid_obj, float dt)
{
  float error = setpoint - measurement;
  float kp = pid_obj.Kp;
  float ki = pid_obj.Ki;
  float kd = pid_obj.Kd;
  
  const float threshold = 0.8; // 0.8 RPM
  if (abs(error) < threshold)
    return 0.0f;
  
  const float integral_max = 50;
  pid_obj.integral += (ki * (pid_obj.pre_error + error)* dt ) / 2 ; 
  pid_obj.integral = constrain(pid_obj.integral, -integral_max, integral_max);
  
  float derivative = (kd * (error - pid_obj.pre_error)) / dt;
  
  pid_obj.pre_error = error;
  
  return kp * error + pid_obj.integral + derivative;
}

void setup() 
{
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  encoder_init();

  node.initNode();
  node.subscribe(sub);

  last_speed_ts = millis();
}

void loop() 
{
  node.spinOnce();
  delay(ROS_SPIN_TIME);
  
  unsigned long now = millis();
  if (now - last_speed_ts >= SPEED_WINDOW_MS && desire_RPM_L !=0 && desire_RPM_R !=0) 
  {
    noInterrupts();
    long r_ticks = right_ticks;
    long l_ticks = left_ticks;
    interrupts();

    long dr = r_ticks - last_right_ticks;
    long dl = l_ticks - last_left_ticks;

    last_right_ticks = r_ticks;
    last_left_ticks  = l_ticks;
    last_speed_ts = now;

    float window_s = (float)SPEED_WINDOW_MS / 1000.0f;
    float r_cps = dr / window_s;                 
    float l_cps = dl / window_s;
    meas_right_rpm = (r_cps / PPR) * 60.0f;
    meas_left_rpm = (l_cps / PPR) * 60.0f;

    // print out RPM value
    char buf[48];
    dtostrf(meas_right_rpm, 0, 2, buf);
    /*node.loginfo((String("Right Speed: ") + String(buf)).c_str()); 
    
    dtostrf(meas_left_rpm, 0, 2, buf); 
    node.loginfo((String("Left Speed: ") + String(buf)).c_str());*/
  }
  
  float com_L = 0.0;
  float com_R = 0.0;
  if (now - last_pid_ts >= PID_WINDOW_MS && desire_RPM_L !=0 && desire_RPM_R !=0) 
  {
    com_L = compute_pid(desire_RPM_L, meas_left_rpm, pid_left, now - last_pid_ts);
    com_R = compute_pid(desire_RPM_R, meas_right_rpm, pid_right, now - last_pid_ts);
    
    last_pid_ts = now;
  }
  
  motor_control(2.5 * desire_RPM_L + com_L,  IN1, IN2, ENA);
  motor_control(3.0 * desire_RPM_R + com_R, IN3, IN4, ENB);
}
