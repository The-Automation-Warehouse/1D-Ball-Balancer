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
#define BLUE_LED 10
#define RED_LED 11

// Each value has 10 spaces reserved for it in the EEPROM
float Kp = 0;
float Ki = 0;
float Kd = 0;
String KpStr = "";
String KiStr = "";
String KdStr = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);
int currentLine = 0;
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
  lcd.cursor_on();
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
  displayMenu();

}




void loop() {

  if (menuMode) {

    aState = digitalRead(encoderA);
    if (aState != aLastState) {
      if (doOnce) {     
        if (digitalRead(encoderB) != aState) { 
          encoderPos ++;
          if (editMode) {
            lcd.setCursor(4, 0);
            if (currentLine == 0) {
              Kp -= 1;
              lcd.print(Kp);
            } else if (currentLine == 1) {
              Ki -= 1;
              lcd.print(Ki);
            } else if (currentLine == 2) {
              Kd -= 1;
              lcd.print(Kd);
            } else if (currentLine == 3) {
              menuMode = false;
              editMode = false;
              digitalWrite(buzzer, HIGH);
              delay(50);
              digitalWrite(buzzer, LOW);
              delay(50);
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Balance mode");
              lcd.setCursor(0, 1);
              lcd.print("Click to exit");
            }
          } else {
            scrollDown();
          }
        } else {
          encoderPos --;
          if (editMode) {
            lcd.setCursor(4, 0);
            if (currentLine == 0) {
              Kp += 1;
              lcd.print(Kp);
            } else if (currentLine == 1) {
              Ki += 1;
              lcd.print(Ki);
            } else if (currentLine == 2) {
              Kd += 1;
              lcd.print(Kd);
            } else if (currentLine == 3) {
              menuMode = false;
              editMode = false;
              digitalWrite(buzzer, HIGH);
              delay(50);
              digitalWrite(buzzer, LOW);
              delay(50);
              lcd.clear();
              lcd.cursor_off();
              lcd.setCursor(0, 0);
              lcd.print("Balance mode");
              lcd.setCursor(0, 1);
              lcd.print("Click to exit");
            }
          } else {
            scrollUp();
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
        lcd.setCursor(15, 0);
        lcd.print(" ");
        // Clear the EEPROM
        for (int i = 0; i < 10; i++)
        {
          EEPROM.write(i, 0);
          EEPROM.write(i + 10, 0);
          EEPROM.write(i + 20, 0);
        }
        // Write the data to the EEPROM
        KpStr = String(Kp);
        KiStr = String(Ki);
        KdStr = String(Kd);
        for (unsigned int i = 0; i < KpStr.length(); i++)
        {
          EEPROM.write(i, KpStr[i]);
        }
        for (unsigned int i = 0; i < KiStr.length(); i++)
        {
          EEPROM.write(i + 10, KiStr[i]);
        }
        for (unsigned int i = 0; i < KdStr.length(); i++)
        {
          EEPROM.write(i + 20, KdStr[i]);
        }
        
      } else {
        editMode = true;
        lcd.cursor_on();
        lcd.setCursor(15, 0);
        lcd.print("*");
      }

      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
      delay(50);

      while (digitalRead(encoderSwitch) == LOW) {}
      delay(100);
    }


  } else {
    balance();
  }
}





void balance() {

  if (digitalRead(encoderSwitch) == LOW) {
    menuMode = true;
    editMode = false;
    while (digitalRead(encoderSwitch) == LOW) {}
    lcd.cursor_on();
    displayMenu();
    return;
  }

  // PID variables
  // The whole tray is 240mm long
  // The servo can rotate 180 degrees
  // But its limited to +-45 degrees
  // The zero point is at 90 degrees
  // The target distance is 100mm (acounting for the size of the ball)
  float setpoint = 120;
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

  error = setpoint - distance;
  integral += error;
  derivative = error - lastError;
  output = Kp * error + Ki * integral + Kd * derivative;

  angle = map(output, -100, 100, 45, 135);
  servo.write(angle);


}






void displayMenu() {
  switch (currentLine) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Kp:             ");
      lcd.setCursor(4, 0);
      lcd.print(Kp);
      lcd.setCursor(0, 1);
      lcd.print("Ki:             ");
      lcd.setCursor(4, 1);
      lcd.print(Ki);
      lcd.setCursor(4, 0);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Ki:             ");
      lcd.setCursor(4, 0);
      lcd.print(Ki);
      lcd.setCursor(0, 1);
      lcd.print("Kd:             ");
      lcd.setCursor(4, 1);
      lcd.print(Kd);
      lcd.setCursor(4, 0);
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Kd:             ");
      lcd.setCursor(4, 0);
      lcd.print(Kd);
      lcd.setCursor(0, 1);
      lcd.print("Start?          ");
      lcd.setCursor(4, 0);
      break;
    case 3:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Start?          ");
      lcd.print(" ");
      lcd.setCursor(6, 0);
      break;
  }

  Serial.print("Current menu line: ");
  Serial.println(currentLine);


}






void scrollUp() {

  if (currentLine < 3) {
    currentLine++;
  } else {
    currentLine = 0;
  }

  displayMenu();
}

void scrollDown() {
  
  if (currentLine > 0) {
    currentLine--;
  } else {
    currentLine = 3;
  }

  displayMenu();
}