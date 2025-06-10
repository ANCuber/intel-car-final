#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <Servo.h>
#include <stdlib.h>
#include <SoftwareSerial.h>
#include <stdint.h>

// Accelerometer setup
Adafruit_ADXL345_Unified accel(12345);

// Create servo objects for both axes
Servo servoX;
Servo servoY;
const int servoPinX = 2; 
const int servoPinY = 7;
const float maxTiltAngle = 2.1;
const float thetaRange = 20;
const float theta0 = 40;
const float thetaL = theta0 - thetaRange;
const float thetaR = theta0 + thetaRange;
const float deadband = 0.85;
float currentAngleX = theta0;
float currentAngleY = theta0;
int t = 0;

// Serial I/O
const int32_t baudrate = 115200;

// PID control
float kp = 1;  // Proportional gain
float ki = 0;  // Integral gain
float kd = 0; // Derivative gain

float errorX = 0.0, errorY = 0.0;
float prevErrorX = 0.0, prevErrorY = 0.0;
float integralX = 0.0, integralY = 0.0;

// Add these variables for filtering
float filteredAngleX = 0.0;
float filteredAngleY = 0.0;
const float alpha = 0.8; // Smoothing factor (0 < alpha <= 1)

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

void getAngle() {
    sensors_event_t event;
    filteredAngleX = filteredAngleY = 0.0;
    for (int i = 0; i < 10; i++) {
        accel.getEvent(&event);
        float rawAngleX = 3.8 - calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z);
        float rawAngleY = -2.2 + calculateAngle(event.acceleration.y, event.acceleration.x, event.acceleration.z);

        filteredAngleX = alpha * rawAngleX + (1 - alpha) * filteredAngleX;
        filteredAngleY = alpha * rawAngleY + (1 - alpha) * filteredAngleY;

        delay(5);
    }

    // Serial.print("Filtered Angle X: ");
    // Serial.println(filteredAngleX);
    // Serial.print("Filtered Angle Y: ");
    // Serial.println(filteredAngleY);
}



void adjust() {
    getAngle();

    if (fabs(filteredAngleX) > deadband) {
        currentAngleX = constrain(currentAngleX + filteredAngleX, thetaL, thetaR);
        servoX.write(currentAngleX);
    }
    if (fabs(filteredAngleY) > deadband) {
        currentAngleY = constrain(currentAngleY + filteredAngleY, thetaL, thetaR);
        servoY.write(currentAngleY);
    }
}

void initialize() {
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
    // Serial.print("Servo X angle: ");
    // Serial.println(currentAngleX);
    servoY.write(currentAngleY); 
    // Serial.print("Servo Y angle: ");
    // Serial.println(currentAngleY);

    // Serial.println("Servos initialized, starting in a few seconds.");
    delay(2500);

    for (int i = 0; i < 5; ++i) {
        adjust(); // Initial adjustment based on sensor data
        delay(100);
    }
}

void copy_instruction() {
    Serial.println("G");
}

void wait_to_start() {
    while (1) {
        if (Serial.available()) {
            String message = Serial.readStringUntil('\n');
            break;
        }
    }
    initialize(); 
    copy_instruction();
}

void setup() {
    Serial.begin(baudrate);

    wait_to_start();
}

int deltaX, deltaY;
int beta = 1;
int started = 0;

void read() { // Use ONE and ONLY ONE println()
    while (1) {
        if (Serial.available()) {
            String message = Serial.readStringUntil('\n');
            int commaIndex = message.indexOf(',');

            String part1 = message.substring(0, commaIndex);
            String part2 = message.substring(commaIndex + 1);

            deltaX = part1.toInt();
            deltaY = part2.toInt();

            copy_instruction();
            // Serial.print("First received number: ");
            // Serial.print(deltaX);
            // Serial.print(", Second received number: ");
            // Serial.println(deltaY);
            
            break;
        }
    }
}

void tilt() {
    getAngle();
    float adjustAngleX, adjustAngleY;

    if (fabs(filteredAngleX + beta * deltaX) > deadband) {
        // currentAngleX = constrain(currentAngleX + filteredAngleX + beta * deltaX, thetaL, thetaR);
        // currentAngleX = constrain(currentAngleX + filteredAngleX, thetaL, thetaR);
        adjustAngleX = constrain(currentAngleX + beta * deltaX, thetaL, thetaR);
        servoX.write(adjustAngleX);
    }
    if (fabs(filteredAngleY + beta * deltaY) > deadband) {
        // currentAngleY = constrain(currentAngleY + filteredAngleY + beta * deltaY, thetaL, thetaR);
        // currentAngleY = constrain(currentAngleY + filteredAngleY, thetaL, thetaR);
        adjustAngleY = constrain(currentAngleY + beta * deltaY, thetaL, thetaR);
        servoY.write(adjustAngleY);
    }

    Serial.print("Current Angle X: ");
    Serial.print(adjustAngleX);
    Serial.print(", Current Angle Y: ");
    Serial.println(adjustAngleY);

    delay(100);
}

void test() {
    initialize();
    deltaX = 0;
    deltaY = 10;
    while (1) tilt();
}

int dx[] = {5,5,5,5};
int dy[] = {5,5,5,5};

void loop() {
    // initialize();
    // test();
    // wait_to_start();
    // read();

    t = (t + 1) % 4;

    deltaX = dx[t];
    deltaY = dy[t];

    tilt();

    delay(60);
}
