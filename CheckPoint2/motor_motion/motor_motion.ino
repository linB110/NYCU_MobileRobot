#include <Arduino.h>
#include <stdint.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <ros_motion/pwmValue.h> // self-defined ros msg

const int ENA = 6; // left wheel
const int IN1 = 10;
const int IN2 = 11;
const int ENB = 5; // right wheel
const int IN3 = 8;
const int IN4 = 9;

const int ENCODER_R_A = 2;
const int ENCODER_R_B = 7;
const int ENCODER_L_A = 3;
const int ENCODER_L_B = 13;

#define PWM_TO_RPM 1.25
#define ROS_SPIN_TIME 10
#define CPR 960.0f

volatile int32_t pulses_left = 0;
volatile int32_t pulses_right = 0;

volatile uint32_t ticks = 0;

void encoder_init();
void encoderL_ISR() { pulses_left += digitalRead(ENCODER_L_B) ? 1 : -1; }
void encoderR_ISR() { pulses_right += digitalRead(ENCODER_R_B) ? -1 : 1; }

ISR(TIMER1_COMPA_vect) { ticks++; }
float read_RPM(float cpr, int32_t pulses);
float read_RPM_left(float cpr = CPR);
float read_RPM_right(float cpr = CPR);
  
void motor_control(int pwm, int inA, int inB, int enable);
void motor_acuation(const ros_motion::pwmValue& inputValue);
int PID_control(int input_value, int encoder_rpm);

ros::NodeHandle node;
ros::Subscriber<ros_motion::pwmValue> sub("motor_pwm", &motor_acuation);

void encoder_init()
{
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);  
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP); 
  
  attachInterrupt(0, encoderR_ISR, RISING);
  attachInterrupt(1, encoderL_ISR, RISING);
}

void motor_control(int pwm, int inA, int inB, int enable)
{
    boolean isForward = pwm > 0 ? 1 : 0; 
    int pwmVal = abs(pwm);
   
    digitalWrite(inA, isForward ? HIGH : LOW);
    digitalWrite(inB, isForward ? LOW : HIGH);

    analogWrite(enable, pwmVal);
}

float read_RPM(float cpr, int32_t pulses) 
{
  static uint32_t lastT = 0;
  static int32_t lastP = 0;
  uint32_t now = ticks;
  
  if (now - lastT >= 500) {
    int32_t currentP = pulses;
    
    float rpm = ((currentP - lastP) * 60000.0) / ((now - lastT) * cpr);
    lastP = currentP;
    lastT = now;
    return rpm;
  }
  return -1;
}

float read_RPM_left(float cpr) {
  static uint32_t lastT = 0;
  static int32_t  lastP = 0;
  uint32_t now = ticks;
  if (now - lastT >= 500) {
    noInterrupts();
    int32_t currentP = pulses_left;
    interrupts();
    float rpm = ((currentP - lastP) * 60000.0f) / ((now - lastT) * cpr);
    lastP = currentP;
    lastT = now;
    return rpm;
  }
  return -1.0f;
}

float read_RPM_right(float cpr) {
  static uint32_t lastT = 0;
  static int32_t  lastP = 0;
  uint32_t now = ticks;
  if (now - lastT >= 500) {
    noInterrupts();
    int32_t currentP = pulses_right;
    interrupts();
    float rpm = ((currentP - lastP) * 60000.0f) / ((now - lastT) * cpr);
    lastP = currentP;
    lastT = now;
    return rpm;
  }
  return -1.0f;
}

void motor_acuation(const ros_motion::pwmValue& inputValue)
{    
    motor_control(inputValue.leftPWMValue,  IN1, IN2, ENA);
    motor_control(inputValue.rightPWMValue, IN3, IN4, ENB);
}

int PID_control(int input_value, int encoder_rpm)
{
  float output = 0.0;
  int desired_rpm = input_value * PWM_TO_RPM;  // PWM => RPM
  
  float dt = ROS_SPIN_TIME / 1000.0f;   // ros spin every 10ms
  float kp = 10, ki = 3, kd = 0.5;  // PID parameters
 
  static int pre_error = 0;
  int error = desired_rpm - encoder_rpm;
  
  static float integral = 0.0;
  integral += error * dt;
  
  static float derivative = 0.0;
  derivative = (error - pre_error) / dt;    
  
  output = kp * error + ki * integral + kd * derivative;
  
  if (abs(output) < 10)
    return input_value;
  else 
    return output / PWM_TO_RPM; // RPM => PWM
}

void setup()
{
  encoder_init();
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
    
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 1999;
  TIMSK1 = (1 << OCIE1A);
     
  node.initNode();
  node.subscribe(sub);
}


void loop()
{
  node.spinOnce();
  delay(ROS_SPIN_TIME);
  
  float L_RPM = read_RPM_left(CPR);
  char fbuf[16];
  dtostrf(L_RPM, 0, 2, fbuf);
  char buf[48];
  snprintf(buf, sizeof(buf), "Left RPM: %s", fbuf);
  node.loginfo(buf);
  
  float R_RPM = read_RPM_right(CPR);
  char ebuf[16];
  dtostrf(R_RPM, 0, 2, ebuf);
  char buf2[48];
  snprintf(buf2, sizeof(buf2), "Right RPM: %s", ebuf);
  node.loginfo(buf2);

  delay(500);
}

