//
/*Project 9: Two-Wheel Drive Robot with Modes
Overall Idea: A robot that can work in two modes: obstacle avoidance or Bluetooth control.
Objective: Provide autonomous or manual driving modes for mobile robot.
Description:
● Servo rotates ultrasonic for obstacle avoidance.
● 2 push buttons select mode.
● LCD displays mode.
● Bluetooth receives voice commands.
● Buzzer represents horn when detect obstacle
Required Components:
● 2 Wheel + Motor Driver
● Servo Motor
● Ultrasonic Sensor
● 2 push buttons
● LCD with I2C
● Bluetooth Module
● Buzzer
*/

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

// Pin Definitions

// ULTRASONIC SENSOR
#define TRIG_PIN 8
#define ECHO_PIN 10

// LCD
#define LCD_ADDRESS 0x27
#define BUZZER_PIN 11

// Bluetooth
#define BT_RX_PIN 4
#define BT_TX_PIN 5
#define BT_BAUD_RATE 9600

// Servo
#define SERVO_PIN 12

// Motor Driver Pins
#define LEFT_MOTOR_PIN_FORWARD 3
#define LEFT_MOTOR_PIN_BACKWARD 5
#define RIGHT_MOTOR_PIN_FORWARD 6
#define RIGHT_MOTOR_PIN_BACKWARD 9

// Button Pins
#define BUTTON_MODE_PIN_1 2
#define BUTTON_MODE_PIN_2 3

// constatnts 
unsigned long duration = 0 ; // duration of the ultrasonic pulse
int cm = 0; // distance in cm
bool mode = 0 ; // 0 for obstacle avoidance, 1 for Bluetooth control


// LCD initialzing and objects
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// Servo 
Servo servoMotor;

// bluetooth initializing
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN); // RX, TX

// function to read data from the ultrasonic sensor 

void readDistance(){
    digitalWrite (TRIG_PIN,LOW);
    delayMicroseconds(2);
    digitalWrite (TRIG_PIN,HIGH);
    delayMicroseconds(15);
    digitalWrite (TRIG_PIN,LOW);
    delayMicroseconds(10);
  
    durration=pulseIn(ECHO_PIN,HIGH);
    cm=durration/29/2;
}

// 2 wheel motor model 

// [Fx , Tyaw ] = [1 1; 1 -1] * [FL; FR] 
// [FL; FR] = [1 1; 1 -1]^-1 * [Fx ; Tyaw]
// [FL; FR] = [0.5 0.5; 0.5 -0.5] * [Fx ; Tyaw]
// FL = 0.5 * Fx + 0.5 * Tyaw
// FR = 0.5 * Fx - 0.5 * Tyaw
// where Fx is the forward speed and Tyaw is the turning speed

int input_forces[2] = {0,0}; // [Fx, Tyaw]
int output_forces[2] = {0,0}; // [FL, FR]

void computeMotorForces(int Fx, int Tyaw) {
  // Compute the motor forces based on the input forces
  output_forces[0] = (Fx + Tyaw) / 2; // Left motor force
  output_forces[1] = (Fx - Tyaw) / 2; // Right motor force
}

void controlMotors(){
    // Control the motors based on the computed forces
    if (output_forces[0] > 0) {
        analogWrite(LEFT_MOTOR_PIN_FORWARD, output_forces[0]);
        digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    } else {
        analogWrite(LEFT_MOTOR_PIN_BACKWARD, -output_forces[0]);
        digitalWrite(LEFT_MOTOR_PIN_FORWARD, LOW);
    }
    
    if (output_forces[1] > 0) {
        analogWrite(RIGHT_MOTOR_PIN_FORWARD, output_forces[1]);
        digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
    } else {
        analogWrite(RIGHT_MOTOR_PIN_BACKWARD, -output_forces[1]);
        digitalWrite(RIGHT_MOTOR_PIN_FORWARD, LOW);
    }
}
void move(int speed ) { // range for speed is from -255 to 255
  // Move the robot based on the speed value
  // Positive speed for forward, negative for backward, and 0 for stop
  if (speed > 0 ){  // Move forward
    analogWrite(LEFT_MOTOR_PIN_FORWARD, speed);
    analogWrite(RIGHT_MOTOR_PIN_FORWARD, speed);
    digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
  }
  else if (speed < 0 ){ // Move backward
    analogWrite(LEFT_MOTOR_PIN_BACKWARD, -speed);
    analogWrite(RIGHT_MOTOR_PIN_BACKWARD, -speed);
    digitalWrite(LEFT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_FORWARD, LOW);
  } else { // Stop
    digitalWrite(LEFT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
  }

}

void rotate(int speed) { // range for speed is from -255 to 255
  // Rotate the robot based on the speed value
  // Positive speed for clockwise, negative for counterclockwise, and 0 for stop
  if (speed > 0 ){  // Rotate clockwise
    analogWrite(LEFT_MOTOR_PIN_FORWARD, speed);
    analogWrite(RIGHT_MOTOR_PIN_BACKWARD, speed);
    digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_FORWARD, LOW);
  }
  else if (speed < 0 ){ // Rotate counterclockwise
    analogWrite(LEFT_MOTOR_PIN_BACKWARD, -speed);
    analogWrite(RIGHT_MOTOR_PIN_FORWARD, -speed);
    digitalWrite(LEFT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
  } else { // Stop rotation
    digitalWrite(LEFT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
  }
}

void changeToMode1() {
  mode = 0; // Change to obstacle avoidance mode
  lcd.clear();
  lcd.print("Mode: Obstacle");
}
void changeToMode2() {
  mode = 1; // Change to Bluetooth control mode
  lcd.clear();
  lcd.print("Mode: Bluetooth");
}

// setup 

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  bluetooth.begin(BT_BAUD_RATE);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Mode: Obstacle");

  // Initialize Servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90); // Set initial position

  // Initialize Motor Driver Pins
  pinMode(LEFT_MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_BACKWARD, OUTPUT);

  // Initialize Button Pins
  pinMode(BUTTON_MODE_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_MODE_PIN_2, INPUT_PULLUP);

  // Attach interrupt to buttons
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE_PIN_1), changeToMode1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE_PIN_2), changeToMode2, FALLING);

  // Initialize Buzzer Pin
  pinMode(BUZZER_PIN, OUTPUT);

// Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(1000); // Allow sensor to settle
}

void loop(){
    if (mode == 0) { // Obstacle avoidance mode
        servoMotor.write(90); // Set servo to 90 degrees for ultrasonic sensor
        delay(100); // Wait for servo to stabilize
        readDistance(); // Read distance from ultrasonic sensor
        if (cm < 20) { // If an obstacle is detected within 20 cm
            move(0); // Stop the robot
            delay(200);
            digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
            servoMotor.write(20); // Rotate servo to 20 degrees and check is there an obstacle on the right side
            delay(100); // Wait for servo to stabilize
            digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
            readDistance(); // Read distance from ultrasonic sensor
            if (cm < 20) { // If an obstacle is detected on the right side
                digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
                servoMotor.write(160); // Rotate servo to 160 degrees and check is there an obstacle on the left side
                delay(100); // Wait for servo to stabilize
                digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
                readDistance(); // Read distance from ultrasonic sensor
                if (cm < 20) { // If an obstacle is detected on the left side and the right side
                    digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
                    servoMotor.write(90); // Rotate servo to 90 degrees and check is there an obstacle in front of the robot
                    delay(200);       
                    rotate(-255); // Rotate to avoid the obstacle
                    delay(800); // Wait for 0.5 seconds
                    digitalWrite(BUZZER_PIN,LOW);
                    move(255); // Move forward again
                } else { // If no obstacle on the left side rotate to the left 
                    rotate(-255); // Rotate to avoid the obstacle
                    delay(500); // Wait for 0.5 seconds
                    move(255); // Move forward again
                }
            } else { // If no obstacle on the right side rotate to the right
                rotate(255); // Rotate to avoid the obstacle
                delay(500); // Wait for 0.5 seconds
                move(255); // Move forward again
            }
            
        } else {
            digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
            move(255); // Move forward
        }
    } else if (mode == 1) { // Bluetooth control mode
        if (bluetooth.available()) {
            char command = bluetooth.read(); // Read command from Bluetooth
            if (command == 'F') {
                move(255); // Move forward
            } else if (command == 'B') {
                move(-255); // Move backward
            } else if (command == 'L') {
                rotate(-255); // Rotate left
            } else if (command == 'R') {
                rotate(255); // Rotate right
            } else if (command == 'S') {
                move(0); // Stop the robot
            }
        }
    }
}