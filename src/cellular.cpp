#include "cellular.h"

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    Serial1.println(command);

    long int time = millis();
    while ((time + timeout) > millis())
    {
        while (Serial1.available())
        {
            char c = Serial1.read();
            response += c;
        }
    }
    if (debug)
    {
        Serial.print(response);
    }
    return response;
}