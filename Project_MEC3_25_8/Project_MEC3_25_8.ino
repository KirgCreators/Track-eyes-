#include <ros.h>
#include <std_msgs/String.h>

// --- Motor Pins ---
const int LPUL = 2;
const int LDIR = 3;
const int LENA = 4;
const int RPUL = 5;
const int RDIR = 6;
const int RENA = 7;

// --- Button Pins ---
const int forwardBtn = 8;
const int reverseBtn = 9;

// --- ROS Control Toggle Buttons ---
const int disableRosBtn = 10;
const int enableRosBtn  = 11;

// --- Step timing ---
unsigned long lastStepTime = 0;
unsigned long currentDelay = 3000;
const unsigned long minDelay = 500;
const unsigned long accelStep = 5;
const unsigned long pulseWidth = 50;

ros::NodeHandle nh;

// --- Control flags ---
String command = "";
bool rosMove = false;
bool rosReverse = false;
bool btnMove = false;
bool btnReverse = false;

// --- ROS enable flag ---
bool rosEnabled = true;  // true = ROS messages active

// --- Button state tracking (for toggle debounce) ---
bool lastDisableState = HIGH;
bool lastEnableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;  // ms

// --- ROS Callback ---
void commandCallback(const std_msgs::String &msg) {
  if (!rosEnabled) return;  // ignore ROS commands if disabled

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

  // Buttons
  pinMode(forwardBtn, INPUT_PULLUP);
  pinMode(reverseBtn, INPUT_PULLUP);
  pinMode(disableRosBtn, INPUT_PULLUP);
  pinMode(enableRosBtn, INPUT_PULLUP);

  // Enable motors (HB808C enable = active LOW)
  digitalWrite(LENA, LOW);
  digitalWrite(RENA, LOW);

  // Default direction forward
  digitalWrite(LDIR, HIGH);
  digitalWrite(RDIR, HIGH);
}

void loop() {
  nh.spinOnce();  // process ROS messages if enabled

  unsigned long nowMillis = millis();

  // --- ROS enable/disable switch ---
  bool disableState = digitalRead(disableRosBtn);
  bool enableState = digitalRead(enableRosBtn);

  // Simple debounce
  if (nowMillis - lastDebounceTime > debounceDelay) {
    if (disableState == LOW && lastDisableState == HIGH) {
      rosEnabled = false;   // disable ROS control
      rosMove = false;      // stop ROS motion immediately
      nh.loginfo("ROS control DISABLED (manual mode)");
      lastDebounceTime = nowMillis;
    }
    if (enableState == LOW && lastEnableState == HIGH) {
      rosEnabled = true;    // re-enable ROS control
      nh.loginfo("ROS control ENABLED (ROS mode)");
      lastDebounceTime = nowMillis;
    }
  }

  lastDisableState = disableState;
  lastEnableState = enableState;

  // --- Manual button logic ---
  if (digitalRead(forwardBtn) == LOW) {
    btnMove = true;
    btnReverse = false;
  } else if (digitalRead(reverseBtn) == LOW) {
    btnMove = true;
    btnReverse = true;
  } else {
    btnMove = false;
  }

  // --- Decide control source ---
  bool shouldMove = false;
  bool reverseMode = false;

  if (btnMove) {
    shouldMove = true;
    reverseMode = btnReverse;
  } else if (rosEnabled && rosMove) {  // only move by ROS if enabled
    shouldMove = true;
    reverseMode = rosReverse;
  }

  // --- Apply direction ---
  digitalWrite(LDIR, reverseMode ? LOW : HIGH);
  digitalWrite(RDIR, reverseMode ? LOW : HIGH);
  delayMicroseconds(5);

  // --- Step pulse with acceleration ---
  unsigned long now = micros();

  if (shouldMove) {
    if (currentDelay > minDelay) {
      currentDelay = max(minDelay, currentDelay - accelStep);
    }

    if (now - lastStepTime >= currentDelay) {
      digitalWrite(LPUL, HIGH);
      digitalWrite(RPUL, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(LPUL, LOW);
      digitalWrite(RPUL, LOW);

      lastStepTime = micros();
    }
  } else {
    currentDelay = 3000;
  }
}
