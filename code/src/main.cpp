/*
 * This program controls the 1D Ball Balancer.
 * The user can interact with the program through a 16x2 LCD display and a rotary encoder.
 * The system uses a PID controller to balance the ball, the PID values are stored in the EEPROM and can be changed by the user via the rotary encoder and the LCD display.
 * Feedback is achived through the HC-SR04 ultrasonic sensor.
 * An MG996R servo motor is used to control the angle of the platform vie a crank.
 * 
*/


#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}