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

// Function to estimate ball velocity and position
void updateBallState(float angleX, float angleY, float deltaTime) {
    if (deltaTime > 0) {
        // Calculate velocity based on angle change
        float deltaAngleX = angleX - lastAngleX;
        float deltaAngleY = angleY - lastAngleY;
        
        // Update velocity estimates with low-pass filter
        ballVelocityX = ballVelocityX * 0.85 + (deltaAngleX / deltaTime) * 0.15;
        ballVelocityY = ballVelocityY * 0.85 + (deltaAngleY / deltaTime) * 0.15;
        
        // Integrate velocity to estimate position (relative to starting point)
        ballPositionX += ballVelocityX * deltaTime;
        ballPositionY += ballVelocityY * deltaTime;
        
        // Apply decay to position to prevent drift
        ballPositionX *= 0.999;
        ballPositionY *= 0.999;
    }
    
    lastAngleX = angleX;
    lastAngleY = angleY;
}

// Function to calculate required tilt for vector-based control
float calculateVectorTilt(float targetComponent, float currentVelocity, float currentPosition) {
    // If no target vector (targetVectorX and targetVectorY are <0,0>), aim to stop the ball.
    if (abs(targetVectorX) < 0.001 && abs(targetVectorY) < 0.001) {
        return -kd_velocity * currentVelocity - ki_position * currentPosition;
    }
    
    // targetComponent is assumed to be part of a pre-normalized vector from computer vision.
    float currentSpeedInDirection = currentVelocity * (targetComponent > 0 ? 1 : (targetComponent < 0 ? -1 : 0));
    
    float speedError = targetSpeed - abs(currentSpeedInDirection);
    float directionTilt = kp_direction * targetComponent * speedError;
    
    float velocityDamping = -kd_velocity * currentVelocity;
    float positionCorrection = -ki_position * currentPosition;
    float totalTilt = directionTilt + velocityDamping + positionCorrection;
    
    if (abs(currentVelocity) > maxSpeed) {
        totalTilt = -kd_velocity * currentVelocity * 2;
    }
    
    return constrain(totalTilt, -maxTiltAngle, maxTiltAngle);
}

// Function to adjust platform for vector-based control
void adjustPlatform(float platformAngleX, float platformAngleY) {
    int servoAngleX_calc, servoAngleY_calc; // Use local variables for calculation
    bool isInAutoBalance = (abs(targetVectorX) < 0.001 && abs(targetVectorY) < 0.001);
    
    if (isInAutoBalance) {
        servoAngleX_calc = initialAngle - platformAngleX;
        servoAngleY_calc = initialAngle - platformAngleY;
    } else {
        float tiltX = calculateVectorTilt(targetVectorX, ballVelocityX, ballPositionX);
        float tiltY = calculateVectorTilt(targetVectorY, ballVelocityY, ballPositionY);
        servoAngleX_calc = initialAngle + tiltX;
        servoAngleY_calc = initialAngle + tiltY;
    }
    
    // Update global current servo angles for monitoring and control
    currentAngleX = constrain(servoAngleX_calc, 0, 180);
    currentAngleY = constrain(servoAngleY_calc, 0, 180);

    servoX.write(currentAngleX);
    servoY.write(currentAngleY);
}

// Function to process Bluetooth commands
void processBluetoothCommand() {
    if (bluetooth.available()) {
        String command = bluetooth.readStringUntil('\n');
        command.trim();
        
        if (command.startsWith("VECTOR")) {
            int spaceIndex1 = command.indexOf(' ');
            int spaceIndex2 = command.indexOf(' ', spaceIndex1 + 1);
            
            if (spaceIndex1 > 0 && spaceIndex2 > 0) {
                targetVectorX = command.substring(spaceIndex1 + 1, spaceIndex2).toFloat();
                targetVectorY = command.substring(spaceIndex2 + 1).toFloat();
                
                if (abs(targetVectorX) < 0.001 && abs(targetVectorY) < 0.001) {
                    sendBluetoothMessage(bluetooth, "Mode: Auto-balance (target <0,0>)");
                    ballPositionX = 0;
                    ballPositionY = 0;
                } else {
                    sendBluetoothMessageNoLn(bluetooth, "Mode: Vector Control. Target: X=");
                    sendBluetoothMessageNoLn(bluetooth, String(targetVectorX));
                    sendBluetoothMessageNoLn(bluetooth, " Y=");
                    sendBluetoothMessage(bluetooth, String(targetVectorY));
                }
            }
        }
        else if (command.startsWith("STATUS")) {
            bool isInAutoBalance = (abs(targetVectorX) < 0.001 && abs(targetVectorY) < 0.001);
            printBluetoothStatus(bluetooth, isInAutoBalance, targetVectorX, targetVectorY, 
                                 ballVelocityX, ballVelocityY, ballPositionX, ballPositionY, 
                                 targetSpeed, currentAngleX, currentAngleY, 
                                 kp_direction, kd_velocity, ki_position, maxTiltAngle, maxSpeed);
        }
    }
}

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);
    
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
    
    bool isInAutoBalance = (abs(targetVectorX) < 0.001 && abs(targetVectorY) < 0.001);
    printSerialStatus(isInAutoBalance, targetVectorX, targetVectorY, ballVelocityX, ballVelocityY, platformAngleX_val, platformAngleY_val, currentAngleX, currentAngleY);
    
    delay(15);
}