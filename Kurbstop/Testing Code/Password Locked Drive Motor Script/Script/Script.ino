// Code for forwards and backwards drive on 2 motors with password locking

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

const int pwmPin1 = 27; // Signal output pin for Motor 1
const int pwmPin2 = 26; // Signal output pin for Motor 2
const int servoPin = 4; // Signal output pin for Servo
const int channel1 = 0; // LEDC channel for PWM of Motor 1
const int channel2 = 1; // LEDC channel for PWM of Motor 2

// Pulse width values in microseconds for ESC control
const int off = 1500;  // Neutral (stop)
const int fwd = 2000;  // Full forward
const int rev = 1000;  // Full reverse

// Servo Min and Max
const int servoMin = 90;
const int servoMax = 125;
const int servoFlip = 145;

int xPressCount = 0; // Counter to track number of "X" presses
bool isControlEnabled = false; // Flag to enable/disable robot control
const Button password[] = {cross, square, triangle, circle}; // This is the password array, set it to whatever password you want 

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

  // Setup PWM for Motor 1
  ledcSetup(channel1, 50, 16);   // 50 Hz frequency, 16-bit resolution
  ledcAttachPin(pwmPin1, channel1); // Attach PWM channel to pin

  // Setup PWM for Motor 2
  ledcSetup(channel2, 50, 16);   // 50 Hz frequency, 16-bit resolution
  ledcAttachPin(pwmPin2, channel2); // Attach PWM channel to pin

  // Servo Setup
  ESP32PWM::allocateTimer(2);
  servo.setPeriodHertz(200);
  servo.attach(servoPin, 500, 2400);

  // Send neutral signal initially
  sendPWMSignal(channel1, off);
  sendPWMSignal(channel2, off);
  servo.write(servoMin);
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < 1; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      xPressCount = 0;  // Reset the X press counter on connection
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

unsigned long lastServoFlipTime = 0; // Track the time when the servo was flipped
bool servoFlipped = false; // Flag to check if the servo has been flipped

void processGamepad(ControllerPtr gamepad) {
  if(isControlEnabled){
    // Read joystick axes for motor control
    int axisY1 = gamepad->axisY();  // Control Motor 1
    int axisY2 = gamepad->axisRY(); // Control Motor 2

    // Calculate pulse width for Motor 1
    int pulseWidth1 = off; // Default to neutral
    if (axisY1 < 0) {
      pulseWidth1 = 1500 + abs(axisY1); // Forward
    } else if (axisY1 > 0) {
      pulseWidth1 = 1500 - axisY1;     // Reverse
    }

    // Calculate pulse width for Motor 2
    int pulseWidth2 = off; // Default to neutral
    if (axisY2 > 0) {
      pulseWidth2 = 1500 + abs(axisY2); // Forward
    } else if (axisY2 < 0) {
      pulseWidth2 = 1500 - axisY2;     // Reverse
    }

    bool leftBumperPressed = checkLeftBumperPress(gamepad);
    bool rightBumperPressed = checkRightBumperPress(gamepad);
    bool upPressed = checkUpPress(gamepad);

    if(leftBumperPressed) {
      Serial.println("Left Bumper Pressed!");
      servo.write(servoMin);
      servoFlipped = false;  // Ensure servo flip flag is reset
    }
    else if(rightBumperPressed) {
      Serial.println("Right Bumper Pressed!");
      servo.write(servoMax);
      servoFlipped = false;  // Ensure servo flip flag is reset
    }
    else if(upPressed && !servoFlipped) {
      Serial.println("Up Pressed on DPAD!");
      servo.write(servoFlip);
      lastServoFlipTime = millis();  // Record the time when the servo was flipped
      servoFlipped = true;           // Set flag to indicate the servo has been flipped
    }

    // Check if 500 milliseconds have passed since the servo was flipped
    if (servoFlipped && millis() - lastServoFlipTime >= 500) {
      servo.write(servoMin);  // Reset the servo to its original position
      servoFlipped = false;   // Reset the flag
    }

    // Send PWM signals
    sendPWMSignal(channel1, pulseWidth1);
    sendPWMSignal(channel2, pulseWidth2);
  }
  else{
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
    int dutyCycle = map(pulseWidth, 1000, 2000, 3277, 6554);
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
