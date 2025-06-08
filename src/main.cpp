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

// PID control variables for X and Y axes
float kp = 0.4;  // Proportional gain
float ki = 0.01;  // Integral gain
float kd = 0.04; // Derivative gain

float errorX = 0.0, errorY = 0.0;
float prevErrorX = 0.0, prevErrorY = 0.0;
float integralX = 0.0, integralY = 0.0;

// Add these variables for filtering
float filteredAngleX = 0.0;
float filteredAngleY = 0.0;
const float alpha = 0.1; // Smoothing factor (0 < alpha <= 1)

// Function to convert acceleration to degrees
float calculateAngle(float mainAcc, float auxAcc1, float auxAcc2) {
    return atan2(mainAcc, sqrt(auxAcc1 * auxAcc1 + auxAcc2 * auxAcc2)) * 180.0 / PI;
}

// PID control function
float calculatePID(float error, float &prevError, float &integral) {
    integral += error; // Accumulate the integral
    float derivative = error - prevError; // Calculate the derivative
    prevError = error; // Update the previous error
    return (kp * error) + (ki * integral) + (kd * derivative); // PID formula
}

// Function to adjust platform using PID control
void adjustPlatform(float actualPlatformAngleX, float actualPlatformAngleY) {
    const float deadband = 0.5; // Ignore errors smaller than 0.5 degrees

    // Calculate errors (desired angle is 0 for balancing)
    errorX = 0.0 - actualPlatformAngleX;
    errorY = 0.0 - actualPlatformAngleY;

    // Apply deadband
    if (abs(errorX) < deadband) errorX = 0.0;
    if (abs(errorY) < deadband) errorY = 0.0;

    // Calculate PID outputs
    float pidOutputX = calculatePID(errorX, prevErrorX, integralX);
    float pidOutputY = calculatePID(errorY, prevErrorY, integralY);
    
    Serial.print("PID Output X: ");
    Serial.println(pidOutputX);
    Serial.print("PID Output Y: ");
    Serial.println(pidOutputY);

    // Calculate new servo angles
    currentAngleX = constrain(90 + pidOutputX, 80, 100); // Constrain to servo limits
    currentAngleY = constrain(90 + pidOutputY, 80, 100); // Constrain to servo limits

    // Write new angles to servos
    servoX.write(currentAngleX);
    servoY.write(currentAngleY);
    delay(100);
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    bluetooth.begin(9600);

    // Initialize the sensor
    if (!accel.begin()) {
        Serial.println("No ADXL345 sensor detected!");
        while (1); // Halt if sensor not found
    }
    
    // Set the range to +-2G (can be adjusted if needed)
    accel.setRange(ADXL345_RANGE_2_G);
    
    // Initialize servos
    servoX.attach(servoPinX);
    servoY.attach(servoPinY);
    servoX.write(currentAngleX); 
    servoY.write(currentAngleY); 
    delay(4000); 
}

void loop() {
    sensors_event_t event;
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
    
    // Adjust platform based on accelerometer readings and any processed command
    accel.getEvent(&event); // Get a new sensor event

    // Calculate platform angles
    float rawAngleX = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    float rawAngleY = -calculateAngle(event.acceleration.y, event.acceleration.x, event.acceleration.z);

    // Apply low-pass filter
    for (int i = 0; i < 10; i++) {
        filteredAngleX = alpha * rawAngleX + (1 - alpha) * filteredAngleX;
        filteredAngleY = alpha * rawAngleY + (1 - alpha) * filteredAngleY;
        delay(1);
    }

    // Adjust platform using filtered angles
    adjustPlatform(filteredAngleX, filteredAngleY);

    delay(1500); // Loop delay
}