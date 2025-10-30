#include "motor_motion.h"

// ---------- Motor pins ----------
const int ENA = 6; // right wheel (PWM)
const int IN1 = 10;
const int IN2 = 11;
const int ENB = 5; // left wheel (PWM)
const int IN3 = 8;
const int IN4 = 9;

// ---------- Encoder pins ----------
const int ENCODER_R_A = 2;
const int ENCODER_R_B = 7;
const int ENCODER_L_A = 3;
const int ENCODER_L_B = 12;

// ---------- Encoder counters ----------
volatile long right_ticks = 0;
volatile long left_ticks  = 0;

// ---------- Speed tracking ----------
unsigned long last_left_speed_ts  = 0;
unsigned long last_right_speed_ts = 0;
long last_right_ticks = 0;
long last_left_ticks  = 0;

// ---- ISR ----
void right_wheel_isr();
void left_wheel_isr();

// ==================================================
// initialization
// ==================================================
void motor_encoder_begin()
{
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), right_wheel_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), left_wheel_isr,  RISING);

  last_left_speed_ts  = millis();
  last_right_speed_ts = millis();
}


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

// ==================================================
// read RPM data
// ==================================================
float read_left_rpm()
{

  noInterrupts();
  long l_ticks = left_ticks;
  interrupts();

  long dl = l_ticks - last_left_ticks;
  last_left_ticks = l_ticks;

  const float window_s = float(SPEED_WINDOW_MS) / 1000.0f;
  const float l_cps = dl / window_s;
  const float left_rpm = (l_cps / PPR) * 60.0f;

  last_left_speed_ts = millis();
  return left_rpm;
}

float read_right_rpm()
{
  noInterrupts();
  long r_ticks = right_ticks;
  interrupts();

  long dr = r_ticks - last_right_ticks;
  last_right_ticks = r_ticks;

  const float window_s = float(SPEED_WINDOW_MS) / 1000.0f;
  const float r_cps = dr / window_s;
  const float right_rpm = (r_cps / PPR) * 60.0f;

  last_right_speed_ts = millis();
  return right_rpm;
}


// ==================================================
// motor control
// ==================================================
void motor_control(int pwm, int inA, int inB, int enable)
{
  const bool isForward = (pwm >= 0);
  int pwmVal = constrain(abs(pwm), 0, 255);

  digitalWrite(inA, isForward ? LOW : HIGH);
  digitalWrite(inB, isForward ? HIGH : LOW);

  analogWrite(enable, pwmVal);
}

void move_forward(int pwm)
{
  motor_control(pwm-10, IN1, IN2, ENA);
  motor_control(pwm, IN3, IN4, ENB);
}

void move_backward(int pwm)
{
  motor_control(-pwm+10, IN1, IN2, ENA);
  motor_control(-pwm, IN3, IN4, ENB);
}

void turn_right(int inner_pwm, int outer_pwm)
{
  motor_control(inner_pwm, IN1, IN2, ENA);   
  motor_control(outer_pwm, IN3, IN4, ENB);  
}

void turn_left(int inner_pwm, int outer_pwm)
{
  motor_control(outer_pwm, IN1, IN2, ENA);   
  motor_control(inner_pwm, IN3, IN4, ENB);   
}

void rotate_ccw(int inner_pwm, int outer_pwm)
{
  motor_control(outer_pwm, IN1, IN2, ENA);   
  motor_control(inner_pwm, IN3, IN4, ENB);
}

void rotate_cw(int inner_pwm, int outer_pwm)
{
  motor_control(inner_pwm, IN1, IN2, ENA);   
  motor_control(outer_pwm, IN3, IN4, ENB);
}

void stop_motors()
{
  motor_control(0, IN1, IN2, ENA);
  motor_control(0, IN3, IN4, ENB);
}
