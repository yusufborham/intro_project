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

/* 

1) I2C LCD 

connect vcc to 5v on the breadboard 
connect GND to GND on nthe breadboard 
connect SCL To A5 on Arduino 
Connect SDA ro A4 on Arduino


2) Bluetooth Module 

connect VCC to 5v on the breadboard
connect GND to GND rail on breadboard 
connect TX from bluetooth module to pin 4 on Arduino 
connect RX from bluetooth module to pin 5 on Arduino 

3) push buttons 

connect one push button between GND and pin 2
connect another push button between GND and pin 3



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
#define BUZZER_PIN 12

// Bluetooth
#define BT_RX_PIN 4
#define BT_TX_PIN 5
#define BT_BAUD_RATE 9600

// Servo
#define SERVO_PIN 11

// Motor Driver Pins
#define LEFT_MOTOR_PIN_FORWARD 6
#define LEFT_MOTOR_PIN_BACKWARD 9
#define RIGHT_MOTOR_PIN_FORWARD 3 
#define RIGHT_MOTOR_PIN_BACKWARD 5 

// Button Pins
#define BUTTON_MODE_PIN_1 2
#define BUTTON_MODE_PIN_2 3

// min distance to detect an obstacle
#define MIN_DISTANCE 30 // cm

#define FORWARD_SPEED 65
#define ROTATION_SPEED 60 
#define SPEED_DIF 3

// constatnts 
unsigned long duration = 0 ; // duration of the ultrasonic pulse
int cm = 0; // distance in cm
bool mode = 0 ; // 0 for obstacle avoidance, 1 for Bluetooth control


// // LCD initialzing and objects
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// Servo 
Servo servoMotor;

// bluetooth initializing
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN); // RX, TX

// function to read data from the ultrasonic sensor and calculate the distance
void readSensor(){
    digitalWrite (TRIG_PIN,LOW);
    delayMicroseconds(2);
    digitalWrite (TRIG_PIN,HIGH);
    delayMicroseconds(15);
    digitalWrite (TRIG_PIN,LOW);
    delayMicroseconds(10);
  
    duration=pulseIn(ECHO_PIN,HIGH);
    cm=duration/29/2;
}

// function to read the distance from the ultrasonic sensor and take the average of 5 readings
// to avoid noise and get a more accurate value
void readDistance(){
    // read 5 times and take the average
    int sum = 0;
    for (int i = 0; i < 5; i++) {
        readSensor(); // Read distance from ultrasonic sensor
        sum += cm; // Add the distance to the sum
        delay(50); // Wait for a short time before the next reading
    }
    cm = sum / 5; // Calculate the average distance
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

  if ( abs(output_forces[0]) > 255 || abs(output_forces[1] > 255)){
    // Limit the output forces to the range of -255 to 255
    output_forces[0] *= 255 / max(abs(output_forces[0]), abs(output_forces[1]));
    output_forces[1] *= 255 / max(abs(output_forces[0]), abs(output_forces[1]));
  }
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
    analogWrite(LEFT_MOTOR_PIN_FORWARD,  speed  + SPEED_DIF);
    analogWrite(RIGHT_MOTOR_PIN_FORWARD, speed  - SPEED_DIF);
    digitalWrite(LEFT_MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_BACKWARD, LOW);
  }
  else if (speed < 0 ){ // Move backward
    analogWrite(LEFT_MOTOR_PIN_BACKWARD,  -speed + SPEED_DIF);
    analogWrite(RIGHT_MOTOR_PIN_BACKWARD, -speed - SPEED_DIF);
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
  delay(1000);
  Serial.println("Hello from arduino ");
  //bluetooth.begin(BT_BAUD_RATE);

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

  // //Attach interrupt to buttons
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE_PIN_1), changeToMode1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE_PIN_2), changeToMode2, FALLING);

  // Initialize Buzzer Pin
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(13,OUTPUT);

// Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(1000); // Allow sensor to settle
}

void loop(){
    // Algorithm 
    // Check the mode and perform actions accordingly
    // Move Forward 
    // Rotate Servo to 90 degrees and read distance from ultrasonic sensor
    // If an obstacle is detected within 20 cm, stop the robot and check for obstacles on the right and left sides
    // If an obstacle is detected on the right side, rotate to the left
    // If no obstacle is detected on the right side, rotate to the right
    // If an obstacle is detected on the left side and the right side rotate 180 degrees to avoid the obstacles
    // If no obstacle is detected infront of the robot, move forward
    // If the mode is Bluetooth control, read commands from Bluetooth and control the robot accordingly
    if (mode == 0) {                                    // Obstacle avoidance mode
        move(FORWARD_SPEED);
        servoMotor.write(70);                           // Set servo to 90 degrees for ultrasonic sensor
        delay(20);                                     // Wait for servo to stabilize
        readDistance();                                 // Read distance from ultrasonic sensor
        if (cm < MIN_DISTANCE) {                        // If an obstacle is detected within 20 cm

            move(0);                                    // Stop the robot
            digitalWrite(BUZZER_PIN, HIGH);             // Turn on buzzer
            delay(500) ;
            servoMotor.write(0);                       // Rotate servo to 20 degrees and check is there an obstacle on the right side
            delay(500);                                 // Wait for servo to stabilize
            digitalWrite(BUZZER_PIN, LOW);              // Turn off buzzer
            readDistance();                             // Read distance from ultrasonic sensor

            if (cm < MIN_DISTANCE) {                    // If an obstacle is detected on the right side

                digitalWrite(BUZZER_PIN, HIGH);         // Turn on buzzer
                servoMotor.write(140);                  // Rotate servo to 160 degrees and check is there an obstacle on the left side
                delay(500);                             // Wait for servo to stabilize
                digitalWrite(BUZZER_PIN, LOW);          // Turn off buzzer
                readDistance();                         // Read distance from ultrasonic sensor
                if (cm < MIN_DISTANCE) {                // If an obstacle is detected on the left side and the right side

                    digitalWrite(BUZZER_PIN, HIGH);     // Turn on buzzer
                    digitalWrite(13,HIGH);
                    servoMotor.write(70);               // Rotate servo to 90 degrees and check is there an obstacle in front of the robot
                    delay(700);    
                    digitalWrite(BUZZER_PIN, LOW);      // Turn off buzzer   
                    rotate(ROTATION_SPEED);             // Rotate to avoid the obstacle
                    delay(1000);                         // Wait for 0.5 seconds
                    move(0);
                    delay(500);
                    digitalWrite(13,LOW);
                    move(FORWARD_SPEED);                // Move forward again

                } else {                                // If no obstacle on the left side rotate to the left 
                    rotate(-ROTATION_SPEED);            // Rotate to left avoid the obstacle
                    delay(1000);                        // Wait for 0.5 seconds
                    move(0);                            // Stop the robot
                    delay(500);                         // Wait for 0.2 seconds
                    move(FORWARD_SPEED);                 // Move forward again
                }
            } else {                                    // If no obstacle on the right side rotate to the right
                rotate(ROTATION_SPEED);                 // Rotate to avoid the obstacle
                delay(600);                             // Wait for 0.5 seconds
                move(0);
                delay(500);
                move(FORWARD_SPEED);                              // Move forward again
            }
            
        } else {
            digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
            move(FORWARD_SPEED); // Move forward
        }


     } 
     else if (mode == 1) { // Bluetooth control mode
        if (bluetooth.available()) {
            char command = bluetooth.read(); // Read command from Bluetooth
            if (command == 'F' || command == 'f') {
                move(255); // Move forward
            } else if (command == 'B' || command == 'b') {
                move(-255); // Move backward
            } else if (command == 'L' || command == 'l') {
                rotate(-255); // Rotate left
            } else if (command == 'R' || command == 'r') {
                rotate(255); // Rotate right
            } else if (command == 'S' || command == 's') {
                move(0); // Stop the robot
            }
        }
    } 
    
}