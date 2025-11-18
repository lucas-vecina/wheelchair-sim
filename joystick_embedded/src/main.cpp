#include <Arduino.h>
#include "main.h"
#include "controller.h"

JoystickOutput joystick;
RobotSpeed robotSpeedReference;
String serialData;

int calSamples = 30;
int calRx = 0;
int calRy = 0;

void setup() {
  Serial.begin(9600);
  pinMode(INPUT, SW_PIN);

  // Calibrate joystick
  for(int i=0; i<calSamples; i++) {
    calRx += analogRead(ANALOG_RX_PIN);
    calRy += analogRead(ANALOG_RY_PIN);

    delay(100);
  }

  calRx = (calRx / calSamples) - 512;
  calRy = (calRy / calSamples) - 512;
}

void loop() {
  // read analog X and Y analog values
  joystick.x = analogRead(ANALOG_RX_PIN) - calRx;
  joystick.y = analogRead(ANALOG_RY_PIN) - calRy;

  joystick2robotSpeed(&joystick, &robotSpeedReference);

  // serialData = String(joystick.x) + "," + String(joystick.y);
  serialData = String(robotSpeedReference.v) + "," + String(robotSpeedReference.w);
  Serial.println(serialData);
  
  delay(100);

}
