#include <Arduino.h>
#include <stdint.h>
#include <ros.h>
#include <std_msgs/Float32.h>
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

#define ROS_SPIN_TIME 10
#define CPR 960.0f

volatile int32_t pulses_left = 0;
volatile int32_t pulses_right = 0;

volatile uint32_t ticks = 0;

float rightWheelRPM = 0.0;
float leftWheelRPM = 0.0;
float desiredRPM_R = 0.0;
float desiredRPM_L = 0.0;

void encoder_init();
void encoderL_ISR() { pulses_left += digitalRead(ENCODER_L_B) ? 1 : -1; }
void encoderR_ISR() { pulses_right += digitalRead(ENCODER_R_B) ? -1 : 1; }

ISR(TIMER1_COMPA_vect) { ticks++; }
float read_RPM(float cpr, int32_t pulses);
float read_RPM_left(float cpr = CPR);
float read_RPM_right(float cpr = CPR);
  
void motor_control(int pwm, int inA, int inB, int enable);
void motor_acuation(const ros_motion::pwmValue& inputValue);
float PID_control(float desired_value, float measurement);

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
    pwmVal = constrain(pwmVal, 0, 255);
   
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
  if (now - lastT >= 200) {
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
  if (now - lastT >= 200) {
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
  desiredRPM_L = inputValue.leftPWMValue;
  desiredRPM_R = inputValue.rightPWMValue;
  
  //motor_control(desiredRPM_L, IN1, IN2, ENA);
  //motor_control(desiredRPM_R, IN3, IN4, ENB);
}

float PID_control(float desired_value, float measurement)
{
  float output = 0.0;
  static float preError = 0.0;
  
  float dt = ROS_SPIN_TIME / 1000.0f;   // ros spin every 10ms
  float kp = 1.2, ki = 0, kd = 0;  // PID parameters
  
  float error = desired_value - measurement;
  float threshold = 1.0;   // error within 1 RPM
  
  if (abs(error) < threshold) return 0.0;
  
  float integral = 0.0;
  float derivative = 0.0;
  
  integral = error * dt;
  const float iMax = 255.0f / max(ki, 0.001f);
  integral = constrain(integral, -iMax, iMax);
  
  derivative = (error - preError) / dt;
  preError = error;
  
  return kp * error + ki * integral + kd * derivative;
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
  
  // PID control
  float mR = read_RPM_right(CPR);
  float mL = read_RPM_left(CPR);
  float measure_R = 0.0;
  float measure_L = 0.0;
  if (mL >= 0.0f) measure_L = mL;
  if (mR >= 0.0f) measure_R = mR;
  
  char buf[48];
  dtostrf(measure_L, 0, 2, buf);
  node.loginfo((String("Left RPM: ") + String(buf)).c_str());
  dtostrf(measure_R, 0, 2, buf);
  node.loginfo((String("Right RPM: ") + String(buf)).c_str());
  
  if (mL >= 0.0f){
    float delta_L = PID_control(desiredRPM_L, measure_L);
    motor_control(desiredRPM_L + delta_L, IN1, IN2, ENA);
  }
  
  if (mR >= 0.0f){
    float delta_R = PID_control(desiredRPM_R, measure_R);
    motor_control(desiredRPM_R + delta_R, IN3, IN4, ENB);
  }
  
  delay(200);
}
