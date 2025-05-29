#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h> // For String, Print, etc.

// Forward declaration for SoftwareSerial.
// This is often sufficient if util.h only passes SoftwareSerial by reference/pointer.
// If util.h methods directly call SoftwareSerial methods not in Print, include the full header.
class SoftwareSerial; 

// Function declarations
void printSerialStatus(bool isInAutoBalance, float targetX, float targetY, float velX, float velY, float platX, float platY, int servoX_angle, int servoY_angle);
void printBluetoothStatus(SoftwareSerial& bt, bool isInAutoBalance, float targetX, float targetY, float velX, float velY, float posX, float posY, float targetSpd, int servoX_angle, int servoY_angle, float kp, float kd, float ki, float maxTlt, float maxBallSpd);
void sendBluetoothMessage(SoftwareSerial& bt, const String& message);
void sendBluetoothMessageNoLn(SoftwareSerial& bt, const String& message);
void sendSerialMessage(const String& message);
void sendInitialBluetoothMessages(SoftwareSerial& bt);

#endif // UTIL_H