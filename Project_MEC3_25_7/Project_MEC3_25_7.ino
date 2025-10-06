#include <ros.h>
#include <std_msgs/String.h>

// --- Motor Pins ---
const int LPUL = 2;  // Left Pulse
const int LDIR = 3;  // Left Direction
const int LENA = 4;  // Left Enable

const int RPUL = 5;  // Right Pulse
const int RDIR = 6;  // Right Direction
const int RENA = 7;  // Right Enable

// --- Button Pins ---
const int forwardBtn = 8;
const int reverseBtn = 9;

// --- Step timing ---
unsigned long lastStepTime = 0;
unsigned long currentDelay = 3000;     // start delay between steps (us) -> very slow start
const unsigned long minDelay = 500;    // fastest speed (us) -> smaller = faster
const unsigned long accelStep = 5;     // acceleration smoothness
const unsigned long pulseWidth = 50;   // step pulse width (us), safe for HB808C

ros::NodeHandle nh;

// --- Control flags ---
String command = "";
bool rosMove = false;     // movement via ROS
bool rosReverse = false;  // ROS direction
bool btnMove = false;     // movement via button
bool btnReverse = false;  // button direction

// --- ROS Callback ---
void commandCallback(const std_msgs::String &msg) {
  command = msg.data;
  if (command == "F") {
    rosMove = true;
    rosReverse = false;
  } else if (command == "R") {
    rosMove = true;
    rosReverse = true;
  } else if (command == "S") {
    rosMove = false;
  }
}

ros::Subscriber<std_msgs::String> sub("motor_command", &commandCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  // Motor pins
  pinMode(LPUL, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LENA, OUTPUT);
  pinMode(RPUL, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(RENA, OUTPUT);

  // Button pins
  pinMode(forwardBtn, INPUT_PULLUP);  // buttons wired to GND
  pinMode(reverseBtn, INPUT_PULLUP);

  // Enable motors (HB808C enable is active LOW)
  digitalWrite(LENA, LOW);
  digitalWrite(RENA, LOW);

  // Default direction forward
  digitalWrite(LDIR, HIGH);
  digitalWrite(RDIR, HIGH);
}

void loop() {
  nh.spinOnce();  // process ROS messages

  // --- Button logic ---
  if (digitalRead(forwardBtn) == LOW) {
    btnMove = true;
    btnReverse = false;
  } else if (digitalRead(reverseBtn) == LOW) {
    btnMove = true;
    btnReverse = true;
  } else {
    btnMove = false;
  }

  // --- Decide final control source ---
  bool shouldMove = false;
  bool reverseMode = false;

  if (btnMove) {
    shouldMove = true;
    reverseMode = btnReverse;
  } else if (rosMove) {
    shouldMove = true;
    reverseMode = rosReverse;
  }

  // --- Apply direction (must settle before step) ---
  digitalWrite(LDIR, reverseMode ? LOW : HIGH);
  digitalWrite(RDIR, reverseMode ? LOW : HIGH);
  delayMicroseconds(5);  // HB808C DIR setup time

  // --- Stepper pulse with acceleration ---
  unsigned long now = micros();

  if (shouldMove) {
    // Smooth ramp down (acceleration)
    if (currentDelay > minDelay) {
      currentDelay = max(minDelay, currentDelay - accelStep);
    }

    // Generate step pulse
    if (now - lastStepTime >= currentDelay) {
      digitalWrite(LPUL, HIGH);
      digitalWrite(RPUL, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(LPUL, LOW);
      digitalWrite(RPUL, LOW);

      lastStepTime = micros();
    }
  } else {
    // Reset delay so next move starts slow again
    currentDelay = 3000;
  }
}
