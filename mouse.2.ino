// Input pins:
int rside = A0;          // Right side sensor input
int rfront = A1;         // Front right line sensor input
int lfront = A2;         // Front left sensor input
int lside = A3;          // Left side sensor input
int sens1 = A4;          // Unassigned sensor input 1
int sens2 = A5;          // Unassigned sensor input 2
int fourwayswitch = A6;  // Input from function switch
int battery = A7;        // Input for battery measurement
int Receive = 0;         // Receive pin
int Transmit = 1;        // Transmit pin
int m1encoder1 = 2;      // Motor 1 encoder 1 input (interrupt pin)
int m1encoder2 = 4;      // Motor 1 encoder 2 input
int m2encoder1 = 3;      // Motor 2 encoder 1 input (interrupt pin)
int m2encoder2 = 5;      // Motor 2 encoder 2 input

// Output pins:
int sensorLED1 = 6;   // 1st diagnostic LED on sensor board
int lmotorDIR = 7;    // Left motor direction input
int rmotorDIR = 8;    // Right motor direction input
int lmotorPWM = 9;    // Left motor PWM pin
int rmotorPWM = 10;   // Right motor PWM pin
int sensorLED2 = 11;  // 2nd diagnostic LED on sensor board
int trigger = 12;     // Trigger for sensor LEDs
int LED13 = 13;       // External LED Red

const int SensorCount = 4;      // Number of sensors
int sensorValues[SensorCount];  // Array to store sensor readings

// Encoder counts
volatile int m1encoderCount = 0;
volatile int m2encoderCount = 0;

int m1encoderCountPrev = 0;
int m2encoderCountPrev = 0;

// Constants for the robot
const int pulsesPerRevolution = 360;                       // Example: 360 pulses per revolution of the encoder
const float wheelDiameter = 0.02;                          // Example: Wheel diameter in meters (adjust as necessary)
const float wheelCircumference = wheelDiameter * 3.14159;  // Calculate wheel circumference (meters)

// Encoder last state
int m1lastEncoded = 0;  // Store previous state of encoder 1
int m2lastEncoded = 0;  // Store previous state of encoder 2

// Calibration data
int minValues[SensorCount];  // Stores minimum sensor values
int maxValues[SensorCount];  // Stores maximum sensor values
int threshold[SensorCount];  // Stores threshold values

/*************************************************************************
*  PID control system variables
*************************************************************************/
float Kp = 0.15;  // Proportional control constant
float Ki = 0;   // Integral control constant
float Kd = 0.005;     // Derivative control constant
int P;
int I;
int D;
int PIDvalue, error;
double dt, last_time;
/*************************************************************************
*  Global variables
*************************************************************************/
int previousError = 0;
int motorspeeda, motorspeedb;

void setup() {
  pinMode(sensorLED1, OUTPUT);
  pinMode(lmotorDIR, OUTPUT);
  pinMode(rmotorDIR, OUTPUT);
  pinMode(lmotorPWM, OUTPUT);
  pinMode(rmotorPWM, OUTPUT);
  pinMode(sensorLED2, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(LED13, OUTPUT);
  pinMode(m1encoder1, INPUT);
  pinMode(m1encoder2, INPUT);
  pinMode(m2encoder1, INPUT);
  pinMode(m2encoder2, INPUT);


  Serial.begin(9600);  // Set up serial monitor comms on USB

  // Perform sensor calibration
  calibrate();
  delay(3000);
}

void loop() {
  digitalWrite(trigger, HIGH);  // Turn emitter on
  double now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;

  // Reading the sensor values
  sensorValues[0] = analogRead(lside);   // Left sensor
  sensorValues[1] = analogRead(lfront);  // Left front sensor
  sensorValues[2] = analogRead(rfront);  // Right front sensor
  sensorValues[3] = analogRead(rside);   // Right sensor

  // Print sensor values
  /*
  Serial.print("Sensor Values: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();  // Newline after printing all values
*/
  calculateMotorSpeedFromEncoders();
  // Call PID control to adjust motors (to be implemented)
  linefollow();
  //delay(2000);  // Delay to avoid flooding the serial monitor
}

void linefollow() {
  digitalWrite(sensorLED2, HIGH);
  int error = (sensorValues[1] - sensorValues[2]);

  P = error;
  I = (I + error) * dt;
  D = (error - previousError) / dt;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motorspeeda = 150 + PIDvalue;
  motorspeedb = 150 - PIDvalue;
  /*
  if (motorspeeda > 255) {
    motorspeeda = 255;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb > 255) {
    motorspeedb = 255;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
    }
    */
  motorspeeda = constrain(motorspeeda, 0, 70);  // Constrain between 0 and 90
  motorspeedb = constrain(motorspeedb, 0, 70);  // Constrain between 0 and 90
  forward(motorspeeda, motorspeedb);
  // Print motor speeds
  Serial.print("Motor A Speed: ");
  Serial.print(motorspeeda);
  Serial.print(" | Motor B Speed: ");
  Serial.println(motorspeedb);
}

void calculateMotorSpeedFromEncoders() {
  // Calculate the difference in encoder counts between the motors
  int m1ticks = m1encoderCount - m1encoderCountPrev;
  int m2ticks = m2encoderCount - m2encoderCountPrev;

  // Calculate the motor speeds based on encoder ticks
  float m1speed = (float)m1ticks / pulsesPerRevolution * wheelCircumference;  // m/s
  float m2speed = (float)m2ticks / pulsesPerRevolution * wheelCircumference;  // m/s

  // Update the previous encoder counts for the next loop
  m1encoderCountPrev = m1encoderCount;
  m2encoderCountPrev = m2encoderCount;

  // If there's a difference between the two motor speeds, correct the slower motor
  int speedDifference = m1speed - m2speed;
  if (speedDifference > 0.05) {          // If motor 1 is faster than motor 2
    motorspeedb = motorspeedb + 5;       // Speed up motor 2
  } else if (speedDifference < -0.05) {  // If motor 2 is faster than motor 1
    motorspeeda = motorspeeda + 5;       // Speed up motor 1
  }
}

// Function to calibrate sensor readings
void calibrate() {
  digitalWrite(trigger, HIGH);  // Turn emitter on
  digitalWrite(LED13, HIGH);    //turn on indicator
  Serial.println("Starting calibration...");

  // Initialize min and max values
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = analogRead(getSensorPin(i));
    maxValues[i] = analogRead(getSensorPin(i));
  }

  // Calibration process (spin the robot and collect 3000 readings)
  for (int j = 0; j < 1000; j++) {
    spinRobot();  // Make the robot spin during calibration
    for (int i = 0; i < SensorCount; i++) {
      int sensorReading = analogRead(getSensorPin(i));
      if (sensorReading < minValues[i]) {
        minValues[i] = sensorReading;
      }
      if (sensorReading > maxValues[i]) {
        maxValues[i] = sensorReading;
      }
    }
  }

  // Calculate threshold values
  for (int i = 0; i < SensorCount; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" -> Min: ");
    Serial.print(minValues[i]);
    Serial.print(", Max: ");
    Serial.print(maxValues[i]);
    Serial.print(", Threshold: ");
    Serial.println(threshold[i]);
  }

  forward(0, 0);  // Stop the robot after calibration
  Serial.println("Calibration complete!");
}

// Function to map sensor index to pin numbers
int getSensorPin(int index) {
  switch (index) {
    case 0: return lside;
    case 1: return lfront;
    case 2: return rfront;
    case 3: return rside;
    default: return A0;  // Default case, should never be reached
  }
}

// Function to make the robot spin (for calibration)
void spinRobot() {
  analogWrite(lmotorPWM, 50);  // Left motor forward
  analogWrite(rmotorPWM, 50);  // Right motor backward

  digitalWrite(lmotorDIR, HIGH);  // Set left motor forward
  digitalWrite(rmotorDIR, LOW);   // Set right motor backward

  delay(10);  // Small delay to allow the robot to spin slightly
}

// Function to control motors
void forward(int leftSpeed, int rightSpeed) {
  analogWrite(lmotorPWM, abs(leftSpeed));
  analogWrite(rmotorPWM, abs(rightSpeed));

  digitalWrite(lmotorDIR, leftSpeed > 0 ? LOW : HIGH);
  digitalWrite(rmotorDIR, rightSpeed > 0 ? LOW : HIGH);
}
