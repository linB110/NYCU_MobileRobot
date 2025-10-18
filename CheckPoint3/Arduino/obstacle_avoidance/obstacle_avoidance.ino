#include <Arduino.h>
#include <stdint.h>

#define EI_NOTEXTERNAL  
#include <EnableInterrupt.h>

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

// ---------- Sensor pins ----------
const int light_sensor = A0;
const int touch_sensor = 4;

#define PPR 960.0f            
#define SPEED_WINDOW_MS 500   

volatile long right_ticks = 0;
volatile long left_ticks  = 0;

unsigned long last_speed_ts = 0;
unsigned long last_pid_ts = 0;
long last_right_ticks = 0;
long last_left_ticks  = 0;

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

void touch_isr()
{
  Serial.print("touched");
}

void encoder_init() 
{
  // Encoder
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

  digitalWrite(inA, isForward ? LOW : HIGH);
  digitalWrite(inB, isForward ? HIGH : LOW);

  analogWrite(enable, pwmVal);
}

void setup() 
{
  Serial.begin(9600);
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // snesor
  pinMode(light_sensor, INPUT);
  pinMode(touch_sensor, INPUT);              
  enableInterrupt(touch_sensor, touch_isr, FALLING);

  encoder_init();

  last_speed_ts = millis();
}

void loop() 
{
  //motor_control(0, IN1, IN2, ENA);
  //motor_control(90, IN3, IN4, ENB);

  //int val = analogRead(light_sensor);
  //int val = digitalRead(light_sensor);
  int val = digitalRead(touch_sensor);
  //Serial.print("val = ");

  // light sensor : dark = 1,  bright = 0
  // touch sensor : opened = 1 touched = 0
  //Serial.println(val);

  delay(10000);
}
