// Input pins:
int fourwayswitch = A6;  // Input from function switch (analog pin)

void setup() {
  pinMode(fourwayswitch, INPUT);  // Set the switch pin as input

  Serial.begin(9600);  // Set up serial monitor comms on USB
}

void loop() {
  // Read the raw analog value from the switch (A6 pin)
  int switchState = analogRead(fourwayswitch);

  // Output the raw analog value to Serial Monitor
  Serial.println(switchState);

  // Add a small delay to make the output readable
  delay(500);  // Adjust delay as needed
}