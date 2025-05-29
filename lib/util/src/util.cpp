#include "util.h"
#include <Arduino.h> // For Serial object
#include <SoftwareSerial.h> // For SoftwareSerial object

void printSerialStatus(bool isInAutoBalance, float targetX, float targetY, float velX, float velY, float platX, float platY, int servoX_angle, int servoY_angle) {
    Serial.print("Mode: "); Serial.print(isInAutoBalance ? "AUTO" : "VECTOR");
    if (!isInAutoBalance) {
        Serial.print(" Target: X="); Serial.print(targetX);
        Serial.print(" Y="); Serial.print(targetY);
    }
    Serial.print(" Vel: X="); Serial.print(velX, 2);
    Serial.print(" Y="); Serial.print(velY, 2);
    Serial.print(" PlatAng: X="); Serial.print(platX, 1);
    Serial.print(" Y="); Serial.print(platY, 1);
    Serial.print(" Servo: X="); Serial.print(servoX_angle);
    Serial.print(" Y="); Serial.println(servoY_angle);
}

void printBluetoothStatus(SoftwareSerial& bt, bool isInAutoBalance, float targetX, float targetY, float velX, float velY, float posX, float posY, float targetSpd, int servoX_angle, int servoY_angle, float kp, float kd, float ki, float maxTlt, float maxBallSpd) {
    bt.print("Mode: ");
    bt.println(isInAutoBalance ? "Auto-Balance" : "Vector Control");
    if (!isInAutoBalance) {
        bt.print("Target Vector: X=");
        bt.print(targetX);
        bt.print(" Y=");
        bt.println(targetY);
    }
    bt.print("Ball Velocity: X=");
    bt.print(velX);
    bt.print(" Y=");
    bt.println(velY);
    bt.print("Ball Position: X=");
    bt.print(posX);
    bt.print(" Y=");
    bt.println(posY);
    bt.print("Target Speed (fixed): ");
    bt.println(targetSpd);
    bt.print("Servo Angles: X=");
    bt.print(servoX_angle);
    bt.print(" Y=");
    bt.println(servoY_angle);
    bt.print("Params: KP_dir="); bt.print(kp);
    bt.print(" KD_vel="); bt.print(kd);
    bt.print(" KI_pos="); bt.println(ki);
    bt.print("MaxTilt="); bt.print(maxTlt);
    bt.print(" MaxBallSpeed="); bt.println(maxBallSpd);
}

void sendBluetoothMessage(SoftwareSerial& bt, const String& message) {
    bt.println(message);
}

void sendBluetoothMessageNoLn(SoftwareSerial& bt, const String& message) {
    bt.print(message);
}

void sendSerialMessage(const String& message) {
    Serial.println(message);
}

void sendInitialBluetoothMessages(SoftwareSerial& bt) {
    bt.println("Platform Ready. Default: Auto-Balance.");
    bt.println("Commands: VECTOR X Y, STATUS");
    bt.println("Parameters are fixed in code.");
}