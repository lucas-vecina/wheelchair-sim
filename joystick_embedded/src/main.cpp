#include <Arduino.h>
#include "main.h"
#include "controller.h"
#include "kinematics.h"
#include "pwm.h"
#include "h_bridge.h"

JoystickOutput joystick;
RobotSpeed robotSpeedReference;
MotorSpeed motorSpeedReference;
RobotInfo robotInfo;
PwmValue pwmValue;
HBridge hBridge;

void setup() {
  Serial.begin(9600);
  pinMode(INPUT, SW_PIN);

  robotInfo.L = 0.7;
  robotInfo.R = 0.3;
  robotInfo.Vin = 24;
}

void loop() {
  // read analog X and Y analog values
  joystick.x = analogRead(ANALOG_RX_PIN);
  joystick.y = analogRead(ANALOG_RY_PIN);

  joystick2robotSpeed(&joystick, &robotSpeedReference);

  robotSpeed2motorSpeed(&robotSpeedReference, &motorSpeedReference, &robotInfo);

  motorSpeed2pwm(&motorSpeedReference, &pwmValue);

  pwm2voltage(&pwmValue, &hBridge, &robotInfo);

  // Serial.print("x = ");
  // Serial.print(joystick.x);
  // Serial.print(", y = ");
  // Serial.println(joystick.y);

  // Serial.print("L = ");
  // Serial.print(motorSpeedReference.leftMotor);
  // Serial.print(", R = ");
  // Serial.print(motorSpeedReference.rightMotor);
 
  // Serial.print("L = ");
  // Serial.print(pwmValue.leftBridge.dt);
  // Serial.print(", R = ");
  // Serial.print(pwmValue.rightBridge.dt);

  // Serial.print("L = ");
  // Serial.print(hBridge.leftBridge);
  // Serial.print(", R = ");
  // Serial.println(hBridge.rightBridge);

  String data = String(robotSpeedReference.v) + "," + String(robotSpeedReference.w);

  // Send the data over serial, followed by a newline
  Serial.println(data);
  
  delay(100);

}
