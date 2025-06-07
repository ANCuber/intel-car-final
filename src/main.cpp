#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "util.h" // Include your new utility header

// Assign a unique ID to this sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Create servo objects for both axes
Servo servoX;
Servo servoY;
const int servoPinX = 9;  // X-axis servo pin
const int servoPinY = 10; // Y-axis servo pin
const int initialAngle = 90;  // Starting position (horizontal)
int currentAngleX = initialAngle;
int currentAngleY = initialAngle;

// Bluetooth setup
SoftwareSerial bluetooth(2, 3); // RX, TX pins for HC-05

// Control variables
float targetVectorX = 0.0; // Target direction vector X component (0 for auto-balance)
float targetVectorY = 0.0; // Target direction vector Y component (0 for auto-balance)
float ballVelocityX = 0.0; // Estimated ball velocity X
float ballVelocityY = 0.0; // Estimated ball velocity Y
float ballPositionX = 0.0; // Estimated ball position X (integrated velocity)
float ballPositionY = 0.0; // Estimated ball position Y (integrated velocity)
float lastAngleX = 0.0;
float lastAngleY = 0.0;
unsigned long lastTime = 0;

// Control parameters (will be initialized in setup)
float kp_direction;
float kd_velocity;
float ki_position;
float maxTiltAngle;
float targetSpeed;
float maxSpeed;

// Function to convert acceleration to degrees
float calculateAngle(float x, float y, float z, char axis) {
    if (axis == 'x') {
        return atan2(x, sqrt(y*y + z*z)) * 180.0 / PI;
    } else if (axis == 'y') {
        return atan2(y, sqrt(x*x + z*z)) * 180.0 / PI;
    }
    return 0;
}

void adjustPlatform(float angleX) {
    // Adjust servo angle to compensate for tilt
    // If platform is tilted down (positive angle), servo needs to move up
    int adjustment = -angleX;  // Inverse relationship
    
    // Limit adjustment to prevent extreme movements
    adjustment = constrain(adjustment, -45, 45);
    
    // Calculate new servo position
    // int newAngle = initialAngle + adjustment;
    int newAngle = currentAngle + adjustment;
    newAngle = constrain(newAngle, 0, 180);
    
    // Only move if there's a significant change
    if (abs(currentAngle - newAngle) > 0.5) {
        currentAngle = newAngle;
        servoX.write(currentAngle);
    }
    
    lastAngleX = angleX;
    lastAngleY = angleY;
}

    delay(50);
}

void setup() {
    Serial.begin(9600);
    Serial.println("test!");
    
    // Initialize control parameters
    kp_direction = 1.5;
    kd_velocity = 0.8;
    ki_position = 0.1;
    maxTiltAngle = 10.0; // Max platform tilt in degrees
    targetSpeed = 2.0;   // Desired ball speed when moving
    maxSpeed = 5.0;      // Max allowed ball speed before strong damping

    // Initialize the sensor
    if (!accel.begin()) {
        sendSerialMessage("No ADXL345 sensor detected!");
        while(1);
    }
    
    // Set the range to +-2G
    accel.setRange(ADXL345_RANGE_2_G);
    
    // Initialize servos
    servoX.attach(servoPinX);
    servoY.attach(servoPinY);
    servoX.write(initialAngle);
    servoY.write(initialAngle);
    delay(1000); // Give time for servos to reach initial position
    
    lastTime = millis();
    
    sendSerialMessage("Simplified Ball Balancing Platform Initialized (Fixed Params)");
    sendInitialBluetoothMessages(bluetooth);
}

void loop() {
    sensors_event_t event;
    accel.getEvent(&event);
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    if (deltaTime <= 0) deltaTime = 0.001; 
    lastTime = currentTime;
    
    float platformAngleX_val = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z, 'x');
    float platformAngleY_val = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z, 'y');
    
    updateBallState(platformAngleX_val, platformAngleY_val, deltaTime);
    processBluetoothCommand();
    adjustPlatform(platformAngleX_val, platformAngleY_val);
    
    // Check if platform is level
    if (abs(angleX) < 0.5) {
        Serial.println("X-axis is level!");
    }
    
    delay(15);
}