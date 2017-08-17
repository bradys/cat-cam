#include <Servo.h>
/**
 * Simple arduino sketch to listen to usb serial com for the following:
 * 'r' - Request from com to read the servo angle
 * 'nnn/n' - accepts numbers to set the servo angle followed by newline char
 */

String inString = "";

Servo myservo; 

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

}

void loop() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == '\n') {
      Serial.println(inString.toInt());
      myservo.write(inString.toInt());
      inString = "";
    }
    if (inChar == 'r') {
      Serial.println(myservo.read());
      inString = "";
    }      
  }
}

