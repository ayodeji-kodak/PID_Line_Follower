// Input pins
int rside = A0;
int rfront = A1;
int lfront = A2;
int lside = A3;
int sens1 = A4;
int sens2 = A5;
int Receive = 0;
int Transmit = 1;
int m1encoder1 = 2;
int m1encoder2 = 4;
int m2encoder1 = 3;
int m2encoder2 = 5;

// Output pins
int sensorLED1 = 6;
int lmotorDIR = 7;
int rmotorDIR = 8;
int lmotorPWM = 9;
int rmotorPWM = 10;
int sensorLED2 = 11;
int trigger = 12;
int LED13 = 13;

#define SETPOINT 500  // Adjusted to match weighted average midpoint

const int SensorCount = 2;
int sensorPins[SensorCount] = {lfront, rfront};  // For readLineCustom
int minValues[SensorCount];
int maxValues[SensorCount];
int threshold[SensorCount];

float Kp = 0.01;
float Ki = 0;
float Kd = 0;
int P, I, D;
int PIDvalue;
double dt, last_time;

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

  Serial.begin(9600);
  calibrate();
  delay(3000);
}

void loop() {
  digitalWrite(trigger, HIGH);  // Turn emitter on
  double now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now;

  linefollow();
}

// Custom weighted line reader using calibrated min/max
unsigned int readLineCustom() {
  int normalized[SensorCount];
  unsigned long weightedSum = 0;
  unsigned int total = 0;
  int positions[SensorCount] = {0, 1000};

  for (int i = 0; i < SensorCount; i++) {
    int raw = analogRead(sensorPins[i]);
    int range = maxValues[i] - minValues[i];
    if (range == 0) range = 1;
    int norm = constrain((raw - minValues[i]) * 1000 / range, 0, 1000);
    normalized[i] = norm;
    weightedSum += (unsigned long)norm * positions[i];
    total += norm;
  }

  if (total == 0) return 500;  // Fallback to center
  return weightedSum / total;
}

void linefollow() {
  digitalWrite(sensorLED2, HIGH);
  unsigned int position = readLineCustom();
  int error = SETPOINT - position;

  P = error;
  I = (I + error) * dt;
  D = (error - previousError) / dt;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motorspeeda = 150 + PIDvalue;
  motorspeedb = 150 - PIDvalue;

  motorspeeda = constrain(motorspeeda, 0, 50);
  motorspeedb = constrain(motorspeedb, 0, 50);

  forward(motorspeeda, motorspeedb);
}

void calibrate() {
  digitalWrite(trigger, HIGH);
  digitalWrite(LED13, HIGH);
  Serial.println("Starting calibration...");

  for (int i = 0; i < SensorCount; i++) {
    int val = analogRead(sensorPins[i]);
    minValues[i] = val;
    maxValues[i] = val;
  }

  for (int j = 0; j < 700; j++) {
    spinRobot();
    for (int i = 0; i < SensorCount; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  for (int i = 0; i < SensorCount; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;

  }

  forward(0, 0);
  Serial.println("Calibration complete!");
}

void spinRobot() {
  analogWrite(lmotorPWM, 50);
  analogWrite(rmotorPWM, 50);
  digitalWrite(lmotorDIR, HIGH);
  digitalWrite(rmotorDIR, LOW);
  delay(10);
}

void forward(int leftSpeed, int rightSpeed) {
  analogWrite(lmotorPWM, abs(leftSpeed));
  analogWrite(rmotorPWM, abs(rightSpeed));
  digitalWrite(lmotorDIR, leftSpeed > 0 ? LOW : HIGH);
  digitalWrite(rmotorDIR, rightSpeed > 0 ? LOW : HIGH);
}
