#include <Servo.h>

// Define motor pins
int motor1A = 20;   // Motor 1 direction pin A
int motor1B = 1;    // Motor 1 direction pin B
int motor2A = 6;    // Motor 2 direction pin A
int motor2B = 9;    // Motor 2 direction pin B
int motor1PWM = 2;  // PWM pin for controlling speed of motor 1
int motor2PWM = 5;  // PWM pin for controlling speed of motor 2

Servo servo1;  // create servo object to control servo 1
Servo servo2;  // create servo object to control servo 2
Servo servo3;  // create servo object to control servo 3
const int trigPin = 17;
const int echoPin = 16;
bool moving = false;
// Initial Servo Positions
int pos1 = 90;  // Initial position of servo 1 (Base)
int pos2 = 90;  // Initial position of servo 2 (Middle Joint)
int pos3 = 90;  // Initial position of servo 3 (Gripper)

const int MAX_POS1 = 175;
const int MIN_POS1 = 45;
const int MAX_POS2 = 120;
const int MIN_POS2 = 0;
const int MAX_POS3 = 120;
const int MIN_POS3 = 75;

bool moving1 = false;
bool moving2 = false;
bool moving3 = false;

void setup() {
  // Set motor pins as OUTPUT
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor1PWM, OUTPUT);  // Set PWM pins as OUTPUT
  pinMode(motor2PWM, OUTPUT);  // Set PWM pins as OUTPUT
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Attach the servos to pins
  servo1.attach(3);   // attaches the servo on pin 2 to the servo1 object
  servo2.attach(4);   // attaches the servo on pin 3 to the servo2 object
  servo3.attach(22);  // attaches the servo on pin 4 to the servo3 object
  // Start serial communication for debugging
  Serial2.begin(9600);
}

void loop() {
  // Check if a command is available in the serial buffer
  if (Serial2.available() > 0) {
    char input = Serial2.read();  // Read the received command
    Serial2.println(input);

    if (input == 'a') {
      moving = true;
    } else if (input == 'f') {
      moveForward();  // Command to move forward
    } else if (input == 'b') {
      moveBackward();  // Command to move backward
    } else if (input == 'l') {
      turnLeft();  // Command to turn left
    } else if (input == 'r') {
      turnRight();  // Command to turn right
    } else if (input == 's') {
      stopMotors();  // Command to stop motors
    } else if (input == 'c') {
      moving1 = true;
      Serial.println("Base servo moving backward");
      moveServo1backward();  // Move servo 1 backward
    } else if (input == 'd') {
      moving1 = true;
      Serial.println("Base servo moving forward");
      moveServo1forward();  // Move servo 1 forward
    } else if (input == 'g') {
      moving2 = true;
      Serial.println("Middle servo moving forward");
      moveServo2forward();  // Move servo 2 forward
    } else if (input == 'h') {
      moving2 = true;
      Serial.println("Middle servo moving backward");
      moveServo2backward();  // Move servo 2 backward
    } else if (input == 'i') {
      moving3 = true;
      Serial.println("Gripper opening");
      moveServo3forward();  // Move servo 3 forward (open gripper)
    } else if (input == 'j') {
      moving3 = true;
      Serial.println("Gripper closing");
      moveServo3backward();  // Move servo 3 backward (close gripper)
    } else {
      stopMotors();  // Stop motors on invalid command
      Serial.println("Unknown command received.");
    }
  }
  if (moving) {
    // Trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the echo time to determine distance
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;

    // Print distance for debugging
    Serial2.print("Distance: ");
    Serial2.println(distance);
    delay(500);

    if (distance >= 30) {
      // Move forward if no obstacle
      moveForward();
    } else {
      // Stop if obstacle detected
      stopMotors();
      moving = false;
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

// Move robot backward
void moveBackward() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
  analogWrite(motor1PWM, 160);  // Set motor 1 speed
  analogWrite(motor2PWM, 172);  // Set motor 2 speed
  Serial.println("Robot moving backward");
}

// Turn robot right
void turnRight() {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
  analogWrite(motor1PWM, 160);
  analogWrite(motor2PWM, 160);
  Serial.println("Robot turning right");
}

// Turn robot left
void turnLeft() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
  analogWrite(motor1PWM, 160);
  analogWrite(motor2PWM, 160);
  Serial.println("Robot turning left");
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
void stop() {
  moving1 = false;
  moving2 = false;
  moving3 = false; 
  Serial.println("Motor stopped.");
}
// Move servo 1 forward
void moveServo1forward() {
  for (pos1 = servo1.read(); pos1 <= MAX_POS1; pos1 += 1) {
    servo1.write(pos1);
    Serial.print("Position1: ");
    Serial2.println(pos1);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the maximum position
    if (pos1 >= MAX_POS1) {
      Serial2.println("Reached maximum position. Stopping.");
      servo1.write(MAX_POS1);  // Hold at max position
      moving1 = false;
      break;  // Exit the loop
    }
  }
}

// Move servo 1 backward
void moveServo1backward() {
  for (pos1 = servo1.read(); pos1 >= MIN_POS1; pos1 -= 1) {
    servo1.write(pos1);
    Serial.print("Position1: ");
    Serial2.println(pos1);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the minimum position
    if (pos1 <= MIN_POS1) {
      Serial2.println("Reached minimum position. Stopping.");
      servo1.write(MIN_POS1);  // Hold at min position
      moving1 = false;
      break;  // Exit the loop
    }
  }
}

// Move servo 2 forward
void moveServo2forward() {
  for (pos2 = servo2.read(); pos2 <= MAX_POS2; pos2 += 1) {
    servo2.write(pos2);
    Serial.print("Position2: ");
    Serial2.println(pos2);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the maximum position
    if (pos2 >= MAX_POS2) {
      Serial2.println("Reached maximum position. Stopping.");
      servo2.write(MAX_POS2);  // Hold at max position
      moving2 = false;
      break;  // Exit the loop
    }
  }
}

// Move servo 2 backward
void moveServo2backward() {
  for (pos2 = servo2.read(); pos2 >= MIN_POS2; pos2 -= 1) {
    servo2.write(pos2);
    Serial.print("Position2: ");
    Serial2.println(pos2);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the minimum position
    if (pos2 <= MIN_POS2) {
      Serial2.println("Reached minimum position. Stopping.");
      servo2.write(MIN_POS2);  // Hold at min position
      moving2 = false;
      break;  // Exit the loop
    }
  }
}

// Move servo 3 forward (open gripper)
void moveServo3forward() {
  for (pos3 = servo3.read(); pos3 <= MAX_POS3; pos3 += 1) {
    servo3.write(pos3);
    Serial.print("Position3: ");
    Serial2.println(pos3);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the maximum position
    if (pos3 >= MAX_POS3) {
      Serial2.println("Reached maximum position. Stopping.");
      servo3.write(MAX_POS3);  // Hold at max position
      moving3 = false;
      break;  // Exit the loop
    }
  }
}

// Move servo 3 backward (close gripper)
void moveServo3backward() {
  for (pos3 = servo3.read(); pos3 >= MIN_POS3; pos3 -= 1) {
    servo3.write(pos3);
    Serial.print("Position3: ");
    Serial2.println(pos3);
    delay(50);

    // Check if the 'd' command is received while moving
    if (Serial2.available() > 0) {
      char stopCommand = Serial2.read();
      if (stopCommand == 'e') {
        stop();
        break;
      }
    }

    // Stop if we reach the minimum position
    if (pos3 <= MIN_POS3) {
      Serial2.println("Reached minimum position. Stopping.");
      servo3.write(MIN_POS3);  // Hold at min position
      moving3 = false;
      break;  // Exit the loop
    }
  }
}
