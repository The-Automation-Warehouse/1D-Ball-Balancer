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

bool resetBalance = false;

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
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(buzzer, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  // Do one long and two short beeps to indicate that the system is ready
  digitalWrite(buzzer, HIGH);
  digitalWrite(RED_LED, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
  digitalWrite(RED_LED, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  digitalWrite(BLUE_LED, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  digitalWrite(BLUE_LED, LOW);

  delay(1000);

  lcd.clear();
  lcd.cursor_on();
  displayMenu();

}




void loop() {

  if (menuMode) {

    aState = digitalRead(encoderA);
    if (aState != aLastState) {
      if (doOnce) {     
        if (digitalRead(encoderB) == aState) { 
          encoderPos ++;
          if (editMode) {
            lcd.setCursor(4, 0);
            if (currentLine == 0) {
              Kp -= 1;
              lcd.print(Kp);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
            } else if (currentLine == 1) {
              Ki -= 1;
              lcd.print(Ki);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
            } else if (currentLine == 2) {
              Kd -= 1;
              lcd.print(Kd);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
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
            scrollDown();
          }
        } else {
          encoderPos --;
          if (editMode) {
            lcd.setCursor(4, 0);
            if (currentLine == 0) {
              Kp += 1;
              lcd.print(Kp);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
            } else if (currentLine == 1) {
              Ki += 1;
              lcd.print(Ki);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
            } else if (currentLine == 2) {
              Kd += 1;
              lcd.print(Kd);
              lcd.setCursor(4, 0);
              digitalWrite(buzzer, HIGH);
              delay(20);
              digitalWrite(buzzer, LOW);
              delay(20);
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
        doOnce = false;
      } else {
      doOnce = true;
      }
      aLastState = aState;
    } 

    if (digitalRead(encoderSwitch) == LOW) {
      if (editMode) {
        editMode = false;
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
  float setpoint = 8.5;
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

  // The sensor accuracy is not great
  // Last resort bodge

  if (distance < 6)
  {
    resetBalance = true;
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
  }

  if (resetBalance)
  {
    servo.write(75);
    if (distance > 20)
    {
      resetBalance = false;
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    }
  } else {
  
    error = setpoint - distance;
    integral += error;
    derivative = error - lastError;
    output = Kp * error + Ki * integral + Kd * derivative;

    angle = map(output, -100, 100, 110, 70);

    if (error > 0.5 || error < -0.5)
    {
      servo.write(angle);
    }

  }


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