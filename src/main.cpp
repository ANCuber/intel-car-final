#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <Servo.h>

// Assign a unique ID to this sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Create servo object
Servo servoX;
const int servoPinX = 9;  // Change this to your servo pin
const int initialAngle = 90;  // Starting position (horizontal)
int currentAngle = initialAngle;

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
    if (abs(currentAngle - newAngle) > 1) {
        currentAngle = newAngle;
        servoX.write(currentAngle);
    }

    delay(15);
}

void setup() {
    Serial.begin(9600);
    
    // Initialize the sensor
    if(!accel.begin()) {
        Serial.println("No ADXL345 sensor detected!");
        while(1);
    }
    
    // Set the range to +-4G
    accel.setRange(ADXL345_RANGE_4_G);
    
    // Initialize servo
    servoX.attach(servoPinX);
    servoX.write(initialAngle);  // Set to initial position
    delay(1000);  // Give time for servo to reach position
}

void loop() {
    sensors_event_t event;
    accel.getEvent(&event);
    
    // Calculate angles for X and Y axes
    float angleX = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z, 'x');
    float angleY = calculateAngle(event.acceleration.x, event.acceleration.y, event.acceleration.z, 'y');
    
    // Display the angles
    Serial.print("Angle X: "); Serial.print(angleX); Serial.print("°  ");
    Serial.print("Angle Y: "); Serial.print(angleY); Serial.println("°");
    Serial.print("Servo Angle: "); Serial.println(currentAngle);
    
    // Adjust platform using servo
    adjustPlatform(angleX);
    
    // Check if platform is level
    if (abs(angleX) <= 1.0) {
        Serial.println("X-axis is level!");
    }
    
    Serial.println("------------------------");
    delay(100);  // Shorter delay for more responsive control
}