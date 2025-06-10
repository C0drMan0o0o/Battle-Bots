// Split Stick Drive

#include <Bluepad32.h>
#include <ESP32Servo.h>

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

Servo servo;

// Enum with individual button states
enum Button {
  cross,
  triangle,
  square,
  circle
};

enum GearState {
  High,
  Low
};

const int leftMotorPWMPin = 27; // Signal output pin for Left Motor
const int rightMotorPWMPin = 26; // Signal output pin for Right Motor
const int servoPin = 4; // Signal output pin for Servo
const int channel1 = 0; // LEDC channel for PWM of Left Motor
const int channel2 = 1; // LEDC channel for PWM of Right Motor

// Pulse width values in microseconds for ESC control
const int off = 1500;  // Neutral (stop)
const int fwd = 2000;  // Full forward
const int rev = 1000;  // Full reverse

// Servo Min and Max
const int servoMin = 2;
const int servoMax = 50;
const int servoFlip = 60;

bool isControlEnabled = false; // Flag to enable/disable robot control
const Button password[] = {cross, square, triangle, circle}; // This is the password array, set it to whatever password you want 

// Gear State
GearState gear;

void setup() {
  Serial.begin(9600);

  // Print firmware and MAC address info
  Serial.println("Initializing Bluepad32...");
  Serial.print("Firmware version installed: ");
  Serial.println(BP32.firmwareVersion());

  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Setup PWM for Left Motor
  ledcSetup(channel1, 200, 16);   // 200 Hz frequency, 16-bit resolution
  ledcAttachPin(leftMotorPWMPin, channel1); // Attach PWM channel to pin

  // Setup PWM for Right Motor
  ledcSetup(channel2, 200, 16);   // 200 Hz frequency, 16-bit resolution
  ledcAttachPin(rightMotorPWMPin, channel2); // Attach PWM channel to pin

  // Servo Setup
  ESP32PWM::allocateTimer(2);
  servo.setPeriodHertz(200);
  servo.attach(servoPin, 500, 2400);

  // Send neutral signal initially
  sendPWMSignal(channel1, off);
  sendPWMSignal(channel2, off);
  servo.write(servoMin);
  gear = High;
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < 1; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      isControlEnabled = false;  // Disable control initially
      return;
    }
  }
  Serial.println("CALLBACK: No empty slot for new controller");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < 1; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller disconnected, index=");
      Serial.println(i);
      sendPWMSignal(0, off);
      sendPWMSignal(1, off);
      myControllers[i] = nullptr;
      isControlEnabled = false;  // Disable control when the controller is disconnected
      return;
    }
  }
  Serial.println("CALLBACK: Disconnected controller not found in myControllers");
}

void passwordCheck(ControllerPtr gamepad) {
  static Button inputSequence[sizeof(password) / sizeof(password[0])]; // Store entered password
  static int inputIndex = 0; // Tracks the current position in the password sequence

  // Check if any button from the password sequence is pressed
  if (checkCrossPress(gamepad)) {
    inputSequence[inputIndex] = cross;
  } 
  else if (checkSquarePress(gamepad)) {
    inputSequence[inputIndex] = square;
  } 
  else if (checkCirclePress(gamepad)) {
    inputSequence[inputIndex] = circle;
  } 
  else if (checkTrianglePress(gamepad)) {
    inputSequence[inputIndex] = triangle;
  } 
  else {
    return; // No relevant button pressed
  }

  // Verify the entered button against the password sequence
  if (inputSequence[inputIndex] == password[inputIndex]) {
    inputIndex++; // Move to the next expected button

    if (inputIndex == sizeof(password) / sizeof(password[0])) {
      isControlEnabled = true; // Unlock control if full password is entered
      Serial.println("Password correct! Control enabled.");
      inputIndex = 0; // Reset for future use
    }
  } 
  else {
    Serial.println("Incorrect input! Resetting password attempt.");
    inputIndex = 0; // Reset if wrong button is pressed
  }
}


bool checkLeftBumperPress(ControllerPtr gamepad) {
  static bool wasLeftBumperPressed = false;

  bool isPressed = gamepad->l1();

  if (isPressed && !wasLeftBumperPressed) {
    wasLeftBumperPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasLeftBumperPressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkRightBumperPress(ControllerPtr gamepad) {
  static bool wasRightBumperPressed = false;

  bool isPressed = gamepad->r1();

  if (isPressed && !wasRightBumperPressed) {
    wasRightBumperPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasRightBumperPressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkCrossPress(ControllerPtr gamepad) {
  static bool wasCrossPressed = false;

  bool isPressed = gamepad->a();

  if (isPressed && !wasCrossPressed) {
    wasCrossPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasCrossPressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkSquarePress(ControllerPtr gamepad) {
  static bool wasSquarePressed = false;

  bool isPressed = gamepad->x();

  if (isPressed && !wasSquarePressed) {
    wasSquarePressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasSquarePressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkCirclePress(ControllerPtr gamepad) {
  static bool wasCirclePressed = false;

  bool isPressed = gamepad->b();

  if (isPressed && !wasCirclePressed) {
    wasCirclePressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasCirclePressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkTrianglePress(ControllerPtr gamepad) {
  static bool wasTrianglePressed = false;

  bool isPressed = gamepad->y();

  if (isPressed && !wasTrianglePressed) {
    wasTrianglePressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasTrianglePressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkUpPress(ControllerPtr gamepad) {
  static bool wasUpPressed = false;

  bool isPressed = gamepad->dpad() == DPAD_UP;

  if (isPressed && !wasUpPressed) {
    wasUpPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasUpPressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkLeftPress(ControllerPtr gamepad) {
  static bool wasLeftPressed = false;

  bool isPressed = gamepad->dpad() == DPAD_LEFT;

  if (isPressed && !wasLeftPressed) {
    wasLeftPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasLeftPressed = false; // Reset the state when button is released
  }

  return false;
}

bool checkRightPress(ControllerPtr gamepad) {
  static bool wasRightPressed = false;

  bool isPressed = gamepad->dpad() == DPAD_RIGHT;

  if (isPressed && !wasRightPressed) {
    wasRightPressed = true; // Update the state
    return true;
  } else if (!isPressed) {
    wasRightPressed = false; // Reset the state when button is released
  }

  return false;
}

unsigned long lastServoFlipTime = 0; // Track the time when the servo was flipped
bool servoFlipped = false; // Flag to check if the servo has been flipped

void processGamepad(ControllerPtr gamepad) {
  if(isControlEnabled) {
    // Read left stick Y for throttle, right stick X for turning
    int throttle = -gamepad->axisY();   // Invert so up is forward
    int turn = gamepad->axisRX();

    // Gear scaling
    float scale = (gear == High) ? 1.0 : 0.5;

    // Mix throttle and turn to calculate motor speeds
    int leftSpeed = throttle + turn;
    int rightSpeed = throttle - turn;

    // Clamp the speeds to acceptable range
    leftSpeed = constrain(leftSpeed, -512, 512);
    rightSpeed = constrain(rightSpeed, -512, 512);

    // Convert to pulse widths
    int leftPulseWidth = off + (int)(leftSpeed * scale);
    int rightPulseWidth = off + (int)(rightSpeed * scale);

    // DPAD gear control
    if(checkLeftPress(gamepad)) {
      gear = Low;
    } 
    else if(checkRightPress(gamepad)) {
      gear = High;
    }

    // Servo control
    bool leftBumperPressed = checkLeftBumperPress(gamepad);
    bool rightBumperPressed = checkRightBumperPress(gamepad);
    bool upPressed = checkUpPress(gamepad);

    if(leftBumperPressed) {
      Serial.println("Left Bumper Pressed!");
      servo.write(servoMin);
      servoFlipped = false;
    } 
    else if(rightBumperPressed) {
      Serial.println("Right Bumper Pressed!");
      servo.write(servoMax);
      servoFlipped = false;
    } 
    else if(upPressed && !servoFlipped) {
      Serial.println("Up Pressed on DPAD!");
      servo.write(servoFlip);
      lastServoFlipTime = millis();
      servoFlipped = true;
    }

    if (servoFlipped && millis() - lastServoFlipTime >= 500) {
      servo.write(servoMin);
      servoFlipped = false;
    }

    // Send motor signals
    sendPWMSignal(channel1, leftPulseWidth);
    sendPWMSignal(channel2, rightPulseWidth);
  } 
  else {
    passwordCheck(gamepad);
  }
}

void loop() {
  BP32.update(); // Update controller states

  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      processGamepad(myController);
    }
  }
}

// Function to convert microseconds to duty cycle and send PWM signal
void sendPWMSignal(int channel, int pulseWidth) {
  static int lastPulseWidth[2] = {-1, -1}; // Keep track of the last pulse width sent for both motors

  // Only send the signal if the pulse width has changed
  if (pulseWidth != lastPulseWidth[channel]) {
    int dutyCycle = (pulseWidth * 65536L) / 5000L;
    ledcWrite(channel, dutyCycle);

    // Debug output
    Serial.print("Motor ");
    Serial.print(channel + 1);
    Serial.print(": Pulse width: ");
    Serial.print(pulseWidth);
    Serial.print(" us, Duty cycle: ");
    Serial.println(dutyCycle);

    lastPulseWidth[channel] = pulseWidth; // Update the last sent pulse width
  }
}
