#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Accelerometer setup
Adafruit_ADXL345_Unified accel(12345);

// Create servo objects for both axes
Servo servoX;
Servo servoY;
const int servoPinX = 8; 
const int servoPinY = 9;
const float maxTiltAngle = 2.1;
float currentAngleX = 90;
float currentAngleY = 90;

// Bluetooth setup
SoftwareSerial bluetooth(10, 11); // RX, TX pins for HC-05

// Control variables
float tiltX = 0.0;
float tiltY = 0.0;
unsigned long lastTime = 0;

// Function to convert acceleration to degrees
float calculateAngle(float mainAcc, float auxAcc1, float auxAcc2) {
    return atan2(mainAcc, sqrt(auxAcc1 * auxAcc1 + auxAcc2 * auxAcc2)) * 180.0 / PI;
}

// Function to adjust platform based on accelerometer and commands
void adjustPlatform(float actualPlatformAngleX, float actualPlatformAngleY) {
    float deltaThetaX = (tiltX - actualPlatformAngleX);
    float deltaThetaY = (tiltY - actualPlatformAngleY);


    Serial.print("(cur, target) X: (");
    Serial.print(currentAngleX);
    currentAngleX = constrain(currentAngleX + 0.6 * deltaThetaX, 80, 100);
    Serial.print(", ");
    Serial.print(currentAngleX);
    Serial.print(") Y: (");
    Serial.print(currentAngleY);
    Serial.print(", ");
    currentAngleY = constrain(currentAngleY + 0.6 * deltaThetaY, 80, 100);
    Serial.print(currentAngleY);
    Serial.println(")");



    servoX.write(constrain(currentAngleX, 80, 100));
    servoY.write(constrain(currentAngleY, 80, 100));
    delay(20);
}

// Function to process Bluetooth commands
void processBluetoothCommand() {
    static int TTL = 0;
    if (bluetooth.available()) {
        String command = bluetooth.readStringUntil('\n');
        command.trim();
        
        if (command.startsWith("VECTOR")) {
            int spaceIndex1 = command.indexOf(' ');
            int spaceIndex2 = command.indexOf(' ', spaceIndex1 + 1);
            
            if (spaceIndex1 > 0 && spaceIndex2 > 0) {
                tiltX = command.substring(spaceIndex1 + 1, spaceIndex2).toFloat();
                tiltY = command.substring(spaceIndex2 + 1).toFloat();
                // No Bluetooth feedback messages to keep it simple
            }
        }

        // Reset TTL when a command is received
        TTL = 3;
    } else if (TTL == 0) {
        tiltX = 0.0;
        tiltY = 0.0;
    } else {
        TTL--;
    }
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    bluetooth.begin(9600);

    // Initialize the sensor
    if (!accel.begin()) {
        Serial.println("No ADXL345 sensor detected!");
        while(1); // Halt if sensor not found
    }
    
    // Set the range to +-2G (can be adjusted if needed)
    accel.setRange(ADXL345_RANGE_2_G);
    
    // Initialize servos
    servoX.attach(servoPinX);
    servoY.attach(servoPinY);
    servoX.write(currentAngleX); 
    servoY.write(currentAngleY); 
    delay(100); 
    
    // lastTime = millis();
}

void loop() {
    // sensors_event_t event;
    bluetooth.println("Ready to receive commands...");

    if (bluetooth.available()) {
        String command = "";
        command += char(bluetooth.read());
        command.trim();

        bluetooth.print("Received command: ");
        bluetooth.println(command);
        Serial.print("Received command: ");    
        Serial.println(command);
    }
    
    // unsigned long currentTime = millis();
    // lastTime = currentTime;
    // while (false) {
    //     for (int i = 60; i <= 100; i++) {
    //         servoX.write(i);
    //         Serial.print("Servo X Angle: ");
    //         Serial.println(i);
    //         delay(100);
    //     }
    
    //     for (int i = 80; i <= 100; i++) {
    //         servoY.write(i);
    //         Serial.print("Servo Y Angle: ");
    //         Serial.println(i);
    //         delay(100);
    //     }
    
    //     for (int i = 100; i >= 80; i--) {
    //         servoX.write(i);
    //         Serial.print("Servo X Angle: ");
    //         Serial.println(i);
    //         delay(100);
    //     }
    
    //     for (int i = 100; i >= 80; i--) {
    //         servoY.write(i);
    //         Serial.print("Servo Y Angle: ");
    //         Serial.println(i);
    //         delay(100);
    //     }
    // }
    
    
    // Adjust platform based on accelerometer readings and any processed command
    // accel.getEvent(&event); // Get a new sensor event
    // float platformAngleX_val = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    // float platformAngleY_val = -calculateAngle(event.acceleration.y, event.acceleration.x, event.acceleration.z);

    // // processBluetoothCommand();
    // Serial.print("Platform Angle X: ");
    // Serial.println(platformAngleX_val);
    // Serial.print("Platform Angle Y: ");
    // Serial.println(platformAngleY_val);

    // adjustPlatform(platformAngleX_val, platformAngleY_val);
    // tiltX = (i - 2) * 5;

    // tiltX = i - 2;
    // tiltY = j - 2;
    // adjustPlatform(platformAngleX_val, platformAngleY_val);
    // delay(2000);
    
    // delay(15); // Loop delay
}