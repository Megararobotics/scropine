#include <Servo.h>
// Motor Pins Definitions
int motor1A = 20;   // Motor 1 direction pin A
int motor1B = 1;    // Motor 1 direction pin B
int motor2A = 6;    // Motor 2 direction pin A
int motor2B = 9;    // Motor 2 direction pin B
int motor1PWM = 2;  // PWM pin for controlling the speed of motor 1
int motor2PWM = 5;  // PWM pin for controlling the speed of motor 2

// Servo Objects
Servo servo1;  // Servo object for controlling servo 1
Servo servo2;  // Servo object for controlling servo 2
Servo servo3;  // Servo object for controlling servo 3

void setup() {
  // Configure motor control pins as outputs
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  // Attach servo objects to their respective pins
  servo1.attach(3);   // Servo 1 attached to pin 3
  servo2.attach(4);   // Servo 2 attached to pin 4
  servo3.attach(22);  // Servo 3 attached to pin 22

  // Set initial positions of servos to 0 degrees
  servo1.write(90);
  servo2.write(130);
  servo3.write(80);
  // Initialize Serial Communication
  Serial2.begin(9600);
}

void loop() {
  // Check if a command is available in the serial buffer
  if (Serial2.available() > 0) {
    char input = Serial2.read();  // Read the received command
    Serial2.println(input);

    // Use if-else statements to handle different commands
    if (input == 'i') {
      shapei();
    } else if (input == 'l') {
      shapel();  // Command to move backward
    }
  }
}

// Move robot forward
void moveForward() {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(motor1PWM, 168);  // Set motor 1 speed
  analogWrite(motor2PWM, 173);  // Set motor 2 speed
  Serial.println("Robot moving forward");
}

// Stop all motors
void stopMotors() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

void shapel() {
  moveForward();
  delay(1000);
  stopMotors();
  servo1.write(100);
  servo2.write(85);
  servo3.write(80);
  Serial2.println("L shape");
}
void shapei() {
  moveForward();
  delay(1000);
  stopMotors();
  servo1.write(95);
  servo2.write(0);
  servo3.write(80);
  Serial2.println("I shape");
}
