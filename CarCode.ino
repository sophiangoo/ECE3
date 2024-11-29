#include <ECE3.h>

// SENSOR VARIABLES
uint16_t sensorValues[8];
uint16_t sensorMinimums[8] = {534, 558, 606, 582, 511, 558, 606, 630}; // rachel's bot - recalibrated
int sensorMaximums[8] = {2142, 2500, 2500, 2326, 1981, 2465, 2500, 2500}; // rachel's bot - recalibrated
int32_t sensorWeights[8] = { -15, -14, -12, -8, 8, 12, 14, 15};

// PIN VARIABLES
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int LED_RF = 41;
const int right_dir_pin = 30;
const int right_nslp_pin = 11;
const int right_pwm_pin = 39;

// PID VARIABLES
float kp = 0.04;
float kd = 0.035;
int32_t prev_error = 0;
int32_t base_speed = 22;


// Function to read sensors and calculate the error
int32_t calculateError() {
  int32_t error = 0;
  ECE3_read_IR(sensorValues);
  int normalizedValue;

  for (int i = 0; i < 8; i++) {
    //    // Normalization of sensor values
    if (sensorValues[i] < sensorMinimums[i]) {
      sensorMinimums[i] = sensorValues[i];
    }
    if (sensorValues[i] > sensorMaximums[i]) {
      sensorMaximums[i] = sensorValues[i];
    }

    normalizedValue = sensorValues[i];
    normalizedValue -= sensorMinimums[i];
    normalizedValue *= 1000;
    normalizedValue /= (sensorMaximums[i] - sensorMinimums[i]);

    // Sum the weighted sensor readings to calculate the error
    error += (normalizedValue * sensorWeights[i]);
  }

  // Normalize the error
  error /= 24;

  return error;
}

void setup() {
  ECE3_Init();
  Serial.begin(9600); // Serial communication for debugging
  delay(1000);

  // PIN SETUP
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH); // high

  pinMode(LED_RF, OUTPUT);

  resetEncoderCount_left();
  resetEncoderCount_right();
}

int encoder_count = 0;

int averageEncoderCount()
{
  //    int getL = getEncoderCount_left();
  //    int getR = getEncoderCount_right();
  return ((getEncoderCount_left() + getEncoderCount_right()) / 2);
}

bool loopDone = false;
bool archDone = false;
bool harpinDone = false;
bool turnaroundDone = false;

bool secHarpinDone = false;
bool secArchDone = false;
bool secLoopDone = false;
bool finish = false;
int loop_speed = 5;
int time_since_loop = 0;
int time_since_arch = 0;
int time_since_harpin = 0;
int time_since_turnaround = 0;

int loopEncoderCount = 0;
int archEncoderCount = 0;
int harpinEncoderCount = 0;
int turnaroundEncoderCount = 0;
int secHarpinEncoderCount = 0;
int secArchEncoderCount = 0;
int secLoopEncoderCount = 0;

void loop() {
  if (allSensorsBlack() && !loopDone)
  {
    digitalWrite(LED_RF, HIGH);
    // time_since_loop = millis();
    loopTurn();
    loopEncoderCount = encoder_count;
    loopDone = true; digitalWrite(LED_RF, LOW);
  }

  if (encoder_count - loopEncoderCount > 640 && !archDone && loopDone) // 650
  {
    // time_since_arch = millis();
    delay(100);
    base_speed = 18;
    arch();
    // rightForArch();
    archEncoderCount = encoder_count;
    archDone = true;
    digitalWrite(LED_RF, LOW);
    base_speed = 22;
  }

  if (encoder_count - archEncoderCount > 1620 && !harpinDone && archDone)
  {
    // time_since_harpin = millis();
    digitalWrite(LED_RF, HIGH);
    harpin();
    harpinEncoderCount = encoder_count;
    digitalWrite(LED_RF, LOW);
  }

  if (encoder_count - harpinEncoderCount > 2600 && !turnaroundDone && loopDone && archDone && harpinDone) // still need to test if 2600 works
  {
    digitalWrite(LED_RF, HIGH);
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(500);
    // time_since_turnaround = millis();
    turnaround();
    turnaroundEncoderCount = encoder_count;
    turnaroundDone = true;
    digitalWrite(LED_RF, LOW);
  }

  if (encoder_count -  turnaroundEncoderCount > 2600 && !secHarpinDone && turnaroundDone)
  {
    digitalWrite(LED_RF, HIGH);
    secHarpin();
    secHarpinEncoderCount = encoder_count;
    secHarpinDone = true;
    digitalWrite(LED_RF, LOW);
    delay(50);
    // base_speed = 22;
  }

  if (encoder_count - secHarpinEncoderCount > 1700 && !secArchDone && secHarpinDone)
  {
    digitalWrite(LED_RF, HIGH);
    secArch();
    secArchEncoderCount = encoder_count;
    secArchDone = true;
    digitalWrite(LED_RF, LOW);
  }

  if (encoder_count - secArchEncoderCount > 320 && !secLoopDone && secArchDone) // 350
  {
    delay(100);
    digitalWrite(LED_RF, HIGH);
    secLoopTurn();
    secLoopEncoderCount = encoder_count;
    secLoopDone = true;
    digitalWrite(LED_RF, LOW);
  }

  if (encoder_count - secLoopEncoderCount > 680 && !finish && secLoopDone)
  {
    digitalWrite(LED_RF, HIGH);
    finishLine();
    finish = true;
    digitalWrite(LED_RF, LOW);
  }

  delay(50);
  // Calculate the error from the sensors
  int32_t error = calculateError();

  // Calculate the proportional and derivative terms
  int32_t proportional_term = error * kp;
  int32_t derivative = error - prev_error;
  int32_t derivative_term = derivative * kd;

  // Update motor speed with PD control
  int32_t left_motor_speed = base_speed - proportional_term - derivative_term;
  int32_t right_motor_speed = base_speed + proportional_term + derivative_term;

  // Ensure motor speeds stay within valid PWM range (0-255)
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  // Write the motor speeds
  analogWrite(left_pwm_pin, left_motor_speed);
  analogWrite(right_pwm_pin, right_motor_speed);

  // Save the current error and derivative for the next loop iteration
  prev_error = error;

  encoder_count = averageEncoderCount();
  delay(50); // Small delay for stability
}

void loopTurn()
{
  int starting_encoder_count = encoder_count;

  // digitalWrite(LED_RF, HIGH);

  // move forward past the horizontal line
  while (abs(encoder_count - starting_encoder_count) < 135) // 135
  {
    // Calculate the error from the sensors
    int32_t error = calculateError();

    // Calculate the proportional and derivative terms
    int32_t proportional_term = error * kp;
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd;

    int32_t left_motor_speed = base_speed - proportional_term - derivative_term;
    int32_t right_motor_speed = base_speed + proportional_term + derivative_term;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);

    prev_error = error;

    encoder_count = averageEncoderCount(); // update encoder counts
  }

  while (abs(encoder_count - starting_encoder_count) < 600) // was 600
  {
    int32_t error = calculateError();

    int32_t proportional_term = error * kp;
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd;

    // Adjust motor speed to follow a circle:
    // Left motor goes faster than right motor for right circle
    int32_t left_motor_speed = loop_speed + proportional_term + derivative_term;
    int32_t right_motor_speed = loop_speed - proportional_term - derivative_term;

    // Increase the difference between the motors to make a tighter circle
    int32_t circle_factor = 115;  // Controls the tightness of the circle
    right_motor_speed = constrain(right_motor_speed - circle_factor, 0, 255);  // Slow down right motor
    left_motor_speed = constrain(left_motor_speed + circle_factor, 0, 255);   // Speed up left motor

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);

    prev_error = error;

    encoder_count = averageEncoderCount(); // update encoder counts

    delay(50);
  }
}

// Variable to track state of turn
bool inRightTurn = false;
bool inLeftTurn = false;
int turn_speed = 65; // 50
int slow_down_speed = 10;
int right_motor_speed = base_speed;
int left_motor_speed = base_speed;

void arch()
{
  int starting_encoder_count = encoder_count;

  while (abs(encoder_count - starting_encoder_count) <= 240) // 222 227
  {
    int32_t error = calculateError();

    // Calculate the proportional and derivative terms
    int32_t proportional_term = error * kp; // was 0.04
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd; // was 0.035

    int32_t left_motor_speed = base_speed - proportional_term - derivative_term;
    int32_t right_motor_speed = base_speed + proportional_term + derivative_term;

    left_motor_speed -= turn_speed;
    right_motor_speed += turn_speed;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Write the motor speeds
    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);


    digitalWrite(LED_RF, HIGH); 
    delay(50); 

    prev_error = error;
    encoder_count = averageEncoderCount(); 
  }

  while (abs(encoder_count - starting_encoder_count) < 265 && abs(encoder_count - starting_encoder_count) > 240)
  {
    int32_t error = calculateError();

    int32_t proportional_term = error * 0.04; // was 0.04
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * 0.035; // was 0.035

    int32_t left_motor_speed = base_speed + proportional_term + derivative_term;
    int32_t right_motor_speed = base_speed - proportional_term - derivative_term;

    left_motor_speed += turn_speed;
    right_motor_speed -= turn_speed;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);

    digitalWrite(LED_RF, HIGH); 
    delay(50);

    prev_error = error;
    encoder_count = averageEncoderCount(); 
  }
}

void turnaround()
{
  //  int starting_encoder_count = encoder_count;
  //
  //  while (abs(encoder_count - starting_encoder_count) < 42) // 375 encoder counts for 180 turn
  //  {
  //    digitalWrite(left_dir_pin, HIGH); // one motor forward
  //    digitalWrite(right_dir_pin, LOW); // other motor backward
  //    analogWrite(left_pwm_pin, base_speed);
  //    analogWrite(right_pwm_pin, base_speed);
  //    delay(50);
  //    encoder_count = averageEncoderCount();
  //  }
  //  digitalWrite(left_dir_pin, LOW); // back to forward

  digitalWrite(left_dir_pin, HIGH); // one motor forward
  digitalWrite(right_dir_pin, LOW); // other motor backward
  analogWrite(left_pwm_pin, base_speed);
  analogWrite(right_pwm_pin, base_speed);
  delay(3000);
  encoder_count = averageEncoderCount();
  digitalWrite(left_dir_pin, LOW); // back to forward
}

void harpin()
{
  int starting_encoder_count = encoder_count;
  while (abs(encoder_count - starting_encoder_count) < 180) // 170
  {
    digitalWrite(right_dir_pin, HIGH);
    left_motor_speed = constrain(base_speed + 35, 0, 255);
    right_motor_speed = constrain(base_speed, 0, 255);
    analogWrite(left_pwm_pin, left_motor_speed);  // Speed up the left motor
    analogWrite(right_pwm_pin, right_motor_speed); // Slow down the right motor
    delay(50);
    encoder_count = averageEncoderCount();

  }

  digitalWrite(right_dir_pin, LOW);

  delay(50);

  harpinDone = true;
  base_speed = 30; // speed up until second harpin turn?
}

void secHarpin()
{
  int starting_encoder_count = encoder_count;
  while (abs(encoder_count - starting_encoder_count) < 165) // was 170
  {
    digitalWrite(left_dir_pin, HIGH);
    left_motor_speed = constrain(base_speed, 0, 255);
    right_motor_speed = constrain(base_speed + 35, 0, 255);
    analogWrite(left_pwm_pin, left_motor_speed);  // Speed up the left motor
    analogWrite(right_pwm_pin, right_motor_speed); // Slow down the right motor
    delay(50);
    encoder_count = averageEncoderCount();
  }

  digitalWrite(left_dir_pin, LOW);
  // base_speed = 30; // 22
  delay(50);
}

void secArch()
{
  int starting_encoder_count = encoder_count;
  int moreTurnSpeed = 60;

  while (abs(encoder_count - starting_encoder_count) <= 240) // 222 227
  {
    int32_t error = calculateError();

    int32_t proportional_term = error * kp; // was 0.04
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd; // was 0.035

    int32_t left_motor_speed = base_speed + proportional_term + derivative_term;
    int32_t right_motor_speed = base_speed - proportional_term - derivative_term;
    
    left_motor_speed += moreTurnSpeed;
    right_motor_speed -= moreTurnSpeed;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Write the motor speeds
    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);


    // Optional: Visual feedback with LED (for debugging or feedback)
    digitalWrite(LED_RF, HIGH); // Turn on LED for debugging or feedback
    delay(50); // Small delay for stability

    prev_error = error;
    encoder_count = averageEncoderCount(); // update encoder counts
  }

  while (abs(encoder_count - starting_encoder_count) < 265 && abs(encoder_count - starting_encoder_count) > 240)
  {
    int32_t error = calculateError();

    // Calculate the proportional and derivative terms
    int32_t proportional_term = error * 0.04; // was 0.04
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * 0.035; // was 0.035

    int32_t left_motor_speed = base_speed - proportional_term - derivative_term;
    int32_t right_motor_speed = base_speed + proportional_term + derivative_term;

    left_motor_speed -= moreTurnSpeed;
    right_motor_speed += moreTurnSpeed;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Write the motor speeds
    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);


    // Optional: Visual feedback with LED (for debugging or feedback)
    digitalWrite(LED_RF, HIGH); // Turn on LED for debugging or feedback
    delay(50); // Small delay for stability

    prev_error = error;
    encoder_count = averageEncoderCount(); // update encoder counts
  }

}

void secLoopTurn()
{
  int starting_encoder_count = encoder_count;

  // move forward past the horizontal line
  while (abs(encoder_count - starting_encoder_count) < 135) // 135
  {
    // Calculate the error from the sensors
    int32_t error = calculateError();

    // Calculate the proportional and derivative terms
    int32_t proportional_term = error * kp;
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd;

    // Update motor speed with PD control
    int32_t left_motor_speed = base_speed - proportional_term - derivative_term;
    int32_t right_motor_speed = base_speed + proportional_term + derivative_term;

    // Ensure motor speeds stay within valid PWM range (0-255)
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Write the motor speeds
    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);

    // Save the current error and derivative for the next loop iteration
    prev_error = error;

    encoder_count = averageEncoderCount(); // update encoder counts
  }

  //    starting_encoder_count = encoder_count;
  while (abs(encoder_count - starting_encoder_count) < 650) // 600
  {
    // Calculate the error from the sensors
    int32_t error = calculateError();

    // Calculate the proportional and derivative terms
    int32_t proportional_term = error * kp;
    int32_t derivative = error - prev_error;
    int32_t derivative_term = derivative * kd;

    // Adjust motor speed to follow a circle:
    // Right motor goes faster than left motor for left circle
    int32_t left_motor_speed = loop_speed - proportional_term - derivative_term;
    int32_t right_motor_speed = loop_speed + proportional_term + derivative_term;

    // Increase the difference between the motors to make a tighter circle
    int32_t circle_factor = 120;  // Controls the tightness of the circle, adjust as needed
    right_motor_speed = constrain(right_motor_speed + circle_factor, 0, 255);  // Speed up right motor
    left_motor_speed = constrain(left_motor_speed - circle_factor, 0, 255);   // Slow down left motor

    // Ensure motor speeds stay within valid PWM range (0-255)
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    // Write the motor speeds
    analogWrite(left_pwm_pin, left_motor_speed);
    analogWrite(right_pwm_pin, right_motor_speed);

    // Save the current error and derivative for the next loop iteration
    prev_error = error;

    encoder_count = averageEncoderCount(); // update encoder counts
    // digitalWrite(LED_RF, HIGH);
    delay(50);
  }
}

void finishLine()
{
  // Stop car when finished
  base_speed = 0;
  analogWrite(left_pwm_pin, base_speed);
  analogWrite(right_pwm_pin, base_speed);
}

bool allSensorsBlack()
{
  ECE3_read_IR(sensorValues);
  for (int i = 0; i < 8; i++) {
    // Check if each sensor reading is close to its maximum value
    if (sensorValues[i] < (sensorMaximums[i] * 0.40)) { // 0.40
      // If any sensor is below 90% of its maximum, return false
      return false;
    }
  }
  // All sensors are detecting black
  // digitalWrite(LED_RF, HIGH);
  return true;
}
