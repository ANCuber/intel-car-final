#include <Arduino.h>

// ULN2003 control pins
#define IN1 8;
#define IN2 9;
#define IN3 10;
#define IN4 11;

// Step sequence (half-step mode)
int stepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Rotate clockwise one full turn (512 * 8 = 4096 steps)
  for (int i = 0; i < 4096; i++) {
    stepMotor(i % 8);
    delay(2);  // Delay time to adjust motor speed
  }

  delay(1000);  // Pause for 1 second

  // Rotate counterclockwise one full turn
  for (int i = 4095; i >= 0; i--) {
    stepMotor(i % 8);
    delay(2);
  }

  delay(1000);
}

void stepMotor(int step) {
  digitalWrite(IN1, stepSequence[step][0]);
  digitalWrite(IN2, stepSequence[step][1]);
  digitalWrite(IN3, stepSequence[step][2]);
  digitalWrite(IN4, stepSequence[step][3]);
}