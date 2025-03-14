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

// Calibration data
int minValues[SensorCount];  // Stores minimum sensor values
int maxValues[SensorCount];  // Stores maximum sensor values
int threshold[SensorCount];  // Stores threshold values

/*************************************************************************
*  PID control system variables
*************************************************************************/
float Kp = 0.5;  // Proportional control constant
float Ki = 0;  // Integral control constant
float Kd = 0;  // Derivative control constant
int P;
int I;
int D;
int PIDvalue, error;

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
}

void loop() {
  digitalWrite(trigger, HIGH);  // Turn emitter on

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
  
  // Call PID control to adjust motors (to be implemented)
  linefollow();
  delay(2000);  // Delay to avoid flooding the serial monitor
}

void linefollow() {
  int error = (sensorValues[1] - sensorValues[2]);

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motorspeeda = 150 + PIDvalue;
  motorspeedb = 150 - PIDvalue;

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
  forward(motorspeeda, motorspeedb);
  // Print motor speeds
  Serial.print("Motor A Speed: ");
  Serial.print(motorspeeda);
  Serial.print(" | Motor B Speed: ");
  Serial.println(motorspeedb);
}

// Function to calibrate sensor readings
void calibrate() {
  digitalWrite(trigger, HIGH);  // Turn emitter on
  Serial.println("Starting calibration...");

  // Initialize min and max values
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = analogRead(getSensorPin(i));
    maxValues[i] = analogRead(getSensorPin(i));
  }

  // Calibration process (spin the robot and collect 3000 readings)
  for (int j = 0; j < 3000; j++) {
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
  analogWrite(lmotorPWM, 150);  // Left motor forward
  analogWrite(rmotorPWM, 150);  // Right motor backward

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
