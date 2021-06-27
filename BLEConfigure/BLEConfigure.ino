#include <SoftwareSerial.h>
#include "secret_defines.h"

SoftwareSerial BTSerial(10, 11); // RX | TX

void sendCommand(const char* command) {
  Serial.print("Sending command: >");
  Serial.println(command);
  
  BTSerial.println(command);
  delay(500);
  if (BTSerial.available())
  {
    char reply[1000];
    int i = 0;
    while (BTSerial.available())
    {
      char c = BTSerial.read();
      Serial.print(c);
      reply[i] = c;
      i += 1;
    }
    reply[i] = '\0';
    Serial.println(F("END"));
//    Serial.print(reply);
  }
}

void bleLoop() {
  while (Serial.available())
  {
    byte b[1];
    Serial.readBytes(b, 1);
    char c = (char)(b[0]);
    if (c != '<') {
      Serial.print("Unknown character received ");
      Serial.print(c);
      Serial.println("");
    }
    else {
      // ########## Read start tag ##########
      String command = Serial.readStringUntil('>');
      
      // ########## Read payload ##########
      String payload = Serial.readStringUntil(';');

      if (command.equalsIgnoreCase("setting")) {
        int separatorIndex = payload.indexOf('=');
        String key = payload.substring(0, separatorIndex);
        String value = payload.substring(separatorIndex + 1);
        Serial.print("SET ");
        Serial.print(key);
        Serial.print(" = ");
        Serial.println(value);
        
      }
      else if (command.equalsIgnoreCase("data")) {
        Serial.print("DATA ");
        Serial.println(payload);
      }
      else if (command.equalsIgnoreCase("lock")) {
        Serial.print("LOCK ");
        if (payload.equals(SECRET_KEY)) {
          Serial.println("Authorized");
        } else {
          Serial.print(payload);
          Serial.println(" <-- Unauthorized");
        }
      }
    }
  }
}

void loop() {
  bleLoop();
}

void setup()
{
  Serial.begin(9600);
  BTSerial.println("U,9600,N");
  BTSerial.begin(9600);  // HC-05 default speed in AT command more
  Serial.println("Waiting for incoming data");
}
