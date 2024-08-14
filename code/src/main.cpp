#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Servo.h>

#define encoderA 2
#define encoderB 3
#define encoderSwitch 4
#define trigger 7
#define echo 6	
#define buzzer 5
#define SERVO_PIN 9

// Each value has 10 spaces reserved for it in the EEPROM
float Kp = 0;
float Ki = 0;
float Kd = 0;
String KpStr = "";
String KiStr = "";
String KdStr = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);
int currentLine = 0;
int currentMenuLine = 0;
Servo servo;

int aState;
int aLastState;  
volatile int encoderPos = 0;
bool doOnce = true;

bool editMode = false;
bool menuMode = true;

void balance();
void displayMenu();
void scrollUp();
void scrollDown();

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.blink_off();
  lcd.cursor_off();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  EEPROM.begin();
  for (int i = 0; i < 10; i++) {
    KpStr += char(EEPROM.read(i));
    KiStr += char(EEPROM.read(i + 10));
    KdStr += char(EEPROM.read(i + 20));
  }
  Kp = KpStr.toFloat();
  Ki = KiStr.toFloat();
  Kd = KdStr.toFloat();

  servo.attach(SERVO_PIN);
  servo.write(90);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderSwitch, INPUT_PULLUP);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  // Do one long and two short beeps to indicate that the system is ready
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);

  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kp: ");
  lcd.print(Kp);
  lcd.setCursor(0, 1);
  lcd.print("Ki: ");
  lcd.print(Ki);

}




void loop() {

  if (menuMode) {

    aState = digitalRead(encoderA);
    if (aState != aLastState) {
      if (doOnce) {     
        if (digitalRead(encoderB) != aState) { 
          encoderPos ++;
          if (editMode) {
            if (currentMenuLine == 0) {
              Kp += 1;
              lcd.setCursor(4, 0);
              lcd.print(Kp);
            } else if (currentMenuLine == 1) {
              Ki += 1;
              lcd.setCursor(4, 1);
              lcd.print(Ki);
            } else if (currentMenuLine == 2) {
              Kd += 1;
              lcd.setCursor(4, 1);
              lcd.print(Kd);
            }
          } else {
            scrollUp();
          }
        } else {
          encoderPos --;
          if (editMode) {
            if (currentMenuLine == 0) {
              Kp += 1;
              lcd.setCursor(4, 0);
              lcd.print(Kp);
            } else if (currentMenuLine == 1) {
              Ki += 1;
              lcd.setCursor(4, 1);
              lcd.print(Ki);
            } else if (currentMenuLine == 2) {
              Kd += 1;
              lcd.setCursor(4, 1);
              lcd.print(Kd);
            }
          } else {
            scrollDown();
          }
        } 
        Serial.print("Position: ");
        Serial.println(encoderPos);
        doOnce = false;
      } else {
      doOnce = true;
      }
      aLastState = aState;
    } 

    if (digitalRead(encoderSwitch) == LOW) {
      if (editMode) {
        editMode = false;
        lcd.cursor_off();
        lcd.setCursor(15, currentLine);
        lcd.print(" ");
      } else {
        editMode = true;
        lcd.cursor_on();
        lcd.setCursor(15, currentLine);
        lcd.print("*");
      }

      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
      delay(50);

      while (digitalRead(encoderSwitch) == LOW) {}
      delay(200);
    }


  } else {
    balance();
  }
}





void balance() {

  if (digitalRead(encoderSwitch) == LOW) {
    editMode = menuMode;
    lcd.setCursor(15, 1);
    lcd.print("E");
    delay(500);
    return;
  }

  float setpoint = 0;
  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float output = 0;
  float angle = 0;
  float distance = 0;
  float duration = 0;

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 2) / 29.1;

  if (distance < 10) {
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
  }

  angle = map(encoderPos, -100, 100, 0, 180);
  error = setpoint - angle;
  integral += error;
  derivative = error - lastError;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  servo.write(output);

}






void displayMenu() {
  switch (currentMenuLine) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Kp: ");
      lcd.print(Kp);
      lcd.setCursor(0, 1);
      lcd.print("Ki: ");
      lcd.print(Ki);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Ki: ");
      lcd.print(Ki);
      lcd.setCursor(0, 1);
      lcd.print("Kd: ");
      lcd.print(Kd);
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Kd: ");
      lcd.print(Kd);
      lcd.setCursor(0, 1);
      lcd.print("Distance: ");
      break;
  }

  Serial.print("Current menu line: ");
  Serial.println(currentMenuLine);


}






void scrollUp() {
  if (currentMenuLine > 0) {
    currentMenuLine--;
  } else {
    currentMenuLine = 2;
  }

  if (currentLine == 1) {
    currentLine = 0;
  }

  displayMenu();
}

void scrollDown() {
  if (currentMenuLine < 2) {
    currentMenuLine++;
  } else {
    currentMenuLine = 0;
  }

  if (currentLine == 0) {
    currentLine = 1;
  }

  displayMenu();
}