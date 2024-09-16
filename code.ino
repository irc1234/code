// Define pins for motor control
#define ENA 9   // Motor 1 (Left)
#define IN1 10  // Motor 1 (Left Forward)
#define IN2 11  // Motor 1 (Left Backward)
#define IN3 12  // Motor 2 (Right Forward)
#define IN4 13  // Motor 2 (Right Backward)
#define ENB 8   // Motor 2 (Right)

int speedPin = A0;  // Throttle input pin from the FS-i6 (left joystick for speed)
int directionPin = A1;  // Direction input pin from the FS-i6 (right joystick for direction)
int directionPin2 = A2; // Optional: Define an additional pin for left/right

int speedVal = 0;
int directionVal = 0;
int directionVal2 = 0;

void setup() {
  // Setup motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup Serial monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Read throttle and direction from FS-i6
  speedVal = analogRead(speedPin);
  directionVal = analogRead(directionPin);
  directionVal2 = analogRead(directionPin2);

  // Map speed value to motor speed (0-255 for PWM control)
  int motorSpeed = map(speedVal, 0, 1023, 0, 255);

  // Check the right joystick's vertical position for forward and backward motion
  if (directionVal > 600) {
    // Move Forward
    moveForward(motorSpeed);
  } else if (directionVal < 400) {
    // Move Backward
    moveBackward(motorSpeed);
  } 
  // Check the right joystick's horizontal position for left and right motion
  else if (directionVal2 > 600) {  // You can tweak this value based on the FS-i6 readings
    // Turn Right
    turnRight(motorSpeed);
  } else if (directionVal2 < 400) {
    // Turn Left
    turnLeft(motorSpeed);
  } else {
    // Stop the car if no direction input is detected
    stopCar();
  }

  // Debugging information
  Serial.print("Speed Value: ");
  Serial.println(motorSpeed);
  Serial.print("Direction Value: ");
  Serial.println(directionVal);
  Serial.print("Direction Value 2: ");
  Serial.println(directionVal2);

  delay(100);  // Delay for stability
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH); // Left motors forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right motors forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);  // Control left motor speed
  analogWrite(ENB, speed);  // Control right motor speed
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);  // Left motors backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Right motors backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);  // Control left motor speed
  analogWrite(ENB, speed);  // Control right motor speed
}

void turnLeft(int speed) {
  digitalWrite(IN1, LOW);   // Left motors stop
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Right motors forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);      // Stop left motor
  analogWrite(ENB, speed);  // Control right motor speed
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH);  // Left motors forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Right motors stop
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);  // Control left motor speed
  analogWrite(ENB, 0);      // Stop right motor
}

void stopCar() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
