#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AFMotor.h>

// Constants and definitions
const int numServos = 6; // Number of servos
const int maxConfigurations = 10; // Maximum number of storable poses
const int stepDelay = 10; // Delay between each step to slow down the servo movement
const int stepSize = 1;   // The number of degrees to move per step

// Servo channels on the PCA9685
const int servoChannels[numServos] = {0, 4, 8, 9, 12, 13};

// Storage structures
int savedConfigurations[maxConfigurations][numServos];
int currentServoPositions[numServos] = {375, 375, 375, 375, 375, 375}; // Default positions
int configCount = 0;            // Counter for stored poses
bool isPlaying = false;         // Status indicating if poses are being played
bool loopPlayback = false;      // Status indicating if poses should be played in a loop
bool stopPlaying = false;       // Status indicating if playback should be stopped
int currentPoseIndex = 0;       // Index of the current pose during playback

// Motor instances for the rover
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String input = ""; // Holds incoming Bluetooth data

void setup() {
  Serial.begin(9600); // Serial communication for Bluetooth

  // Initialize motors
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  // Initialize servo controller
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz for servos

  // Initialize servo positions
  for (int i = 0; i < numServos; i++) {
    pwm.setPWM(servoChannels[i], 0, currentServoPositions[i]);
  }

  Serial.println("Bluetooth Robot Controller ready. Waiting for commands...");
}

void loop() {
  // Process incoming commands
  if (Serial.available()) {
    input = Serial.readStringUntil('\n'); // Read data from Bluetooth
    input.trim();

    Serial.println("Received command: " + input);
    processCommand(input);
  }

  // Check if robot arm is playing poses
  if (isPlaying && !stopPlaying) {
    if (loopPlayback) {
      playPosesInLoop();
    } else {
      playNextPose();
    }
  }
}

void processCommand(String command) {
  // If playing, ignore all rover commands
  if (isPlaying) {
    if (command == "s") { // Allow stop playback command
      stopPlayingPoses();
    } else {
      Serial.println("Playback in progress. Ignoring command: " + command);
    }
    return;
  }

  // Control rover movement
  if (command == "F") {
    forward();
  } else if (command == "B") {
    backward();
  } else if (command == "L") {
    turnLeft();
  } else if (command == "R") {
    turnRight();
  } else if (command == "S") {
    stopMotors(); // Stop motors immediately on "S" command

  // Control robotic arm
  } else if (command.startsWith("1,")) {
    processServoCommand(command, 0); // Control Servo 1
  } else if (command.startsWith("2,")) {
    processServoCommand(command, 1); // Control Servo 2
  } else if (command.startsWith("3,")) {
    processServoCommand(command, 2); // Control Servo 3
  } else if (command.startsWith("4,")) {
    processServoCommand(command, 3); // Control Servo 4
  } else if (command.startsWith("5,")) {
    processServoCommand(command, 4); // Control Servo 5
  } else if (command.startsWith("6,")) {
    processServoCommand(command, 5); // Control Servo 6

  // Save, play, reset poses
  } else if (command == "v") {
    saveCurrentPose();
  } else if (command == "l") {
    startPlayingPoses();
  } else if (command == "r") {
    resetPoses();
  } else if (command == "s") {
    stopPlayingPoses();
  } else if (command == "o") {
    loopPlayback = true;
    Serial.println("Loop playback enabled.");
  } else if (command == "f") {
    loopPlayback = false;
    Serial.println("Loop playback disabled.");
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void processServoCommand(String command, int servoIndex) {
  int commaIndex = command.indexOf(',');
  if (commaIndex > 0) {
    int position = command.substring(commaIndex + 1).toInt();
    int pwmValue = map(position, 0, 180, 150, 600);
    moveToPositionSmoothly(servoIndex, pwmValue);
  }
}

// Rover motor control
void forward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  Serial.println("Moving forward");
}

void backward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Moving backward");
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  Serial.println("Turning left");
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Turning right");
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  Serial.println("Motors stopped");
}

// Arm servo control
void moveToPositionSmoothly(int servoIndex, int targetPwmValue) {
  int currentPwmValue = currentServoPositions[servoIndex];
  if (targetPwmValue > currentPwmValue) {
    for (int pos = currentPwmValue; pos <= targetPwmValue; pos += stepSize) {
      pwm.setPWM(servoChannels[servoIndex], 0, pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = currentPwmValue; pos >= targetPwmValue; pos -= stepSize) {
      pwm.setPWM(servoChannels[servoIndex], 0, pos);
      delay(stepDelay);
    }
  }
  currentServoPositions[servoIndex] = targetPwmValue;
  Serial.println("Servo " + String(servoIndex + 1) + " set to position: " + String(targetPwmValue));
}

// Pose handling for the arm
void saveCurrentPose() {
  if (configCount < maxConfigurations) {
    for (int i = 0; i < numServos; i++) {
      savedConfigurations[configCount][i] = currentServoPositions[i];
    }
    configCount++;
    Serial.println("Pose saved. Total poses: " + String(configCount));
  } else {
    Serial.println("Memory full. Cannot save pose.");
  }
}

void startPlayingPoses() {
  if (configCount > 0) {
    stopMotors(); // Ensure rover is stopped during playback
    isPlaying = true;
    stopPlaying = false;
    currentPoseIndex = 0;
    Serial.println("Starting pose playback");
  } else {
    Serial.println("No poses saved");
  }
}

void playNextPose() {
  if (currentPoseIndex < configCount) {
    for (int i = 0; i < numServos; i++) {
      moveToPositionSmoothly(i, savedConfigurations[currentPoseIndex][i]);
    }
    delay(1000);
    currentPoseIndex++;
  } else {
    isPlaying = false;
    Serial.println("Pose playback finished");
  }
}

void playPosesInLoop() {
  for (int i = 0; i < configCount; i++) {
    if (stopPlaying) break; // Immediate stop on command
    for (int j = 0; j < numServos; j++) {
      moveToPositionSmoothly(j, savedConfigurations[i][j]);
    }
    delay(1000);
  }
}

void stopPlayingPoses() {
  stopPlaying = true;
  isPlaying = false;
  Serial.println("Pose playback stopped");
}

void resetPoses() {
  configCount = 0;
  isPlaying = false;
  loopPlayback = false;
  currentPoseIndex = 0;
  Serial.println("All poses reset");
}
