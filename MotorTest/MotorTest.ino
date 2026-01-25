/*
 * Motor Test - MicroMouse
 * Tests both motors: forward, backward, turns
 * Uses same pin definitions as MMRF.ino
 */

// ========== Motor Pins ==========
// Left motor (was R)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
// Right motor (was L)
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

#define BUZZER_PIN 17

#define TEST_SPEED 150
#define TEST_DURATION 1000 // ms per test

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(MOTOR_STBY, HIGH);

  tone(BUZZER_PIN, 2000, 100);
  delay(500);

  Serial.println("Motor Test - Serial Control Mode");
  Serial.println("Commands:");
  Serial.println("  R<speed>  - Right motor (e.g., R50 or R-50)");
  Serial.println("  L<speed>  - Left motor (e.g., L30 or L-30)");
  Serial.println("  F<speed>  - Both forward (e.g., F100)");
  Serial.println("  B<speed>  - Both backward (e.g., B50)");
  Serial.println("  M<L>,<R>  - Manual both (e.g., M-50,100)");
  Serial.println("  S         - Stop");
  Serial.println("Speed range: -255 to 255");
  Serial.println("Ready!");
}

// Main loop - reads serial commands and executes motor control
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Remove whitespace
    
    if (cmd.length() > 0) {
      processCommand(cmd);
    }
  }
}

// Process incoming serial commands
void processCommand(String cmd) {
  char type = cmd.charAt(0);
  String value = cmd.substring(1);
  
  // Stop command
  if (type == 'S' || type == 's') {
    setMotors(0, 0);
    Serial.println("STOP");
    return;
  }
  
  // Right motor only
  if (type == 'R' || type == 'r') {
    int speed = value.toInt();
    speed = constrain(speed, -255, 255);
    setMotors(0, speed);
    Serial.print("Right motor: ");
    Serial.println(speed);
    return;
  }
  
  // Left motor only
  if (type == 'L' || type == 'l') {
    int speed = value.toInt();
    speed = constrain(speed, -255, 255);
    setMotors(speed, 0);
    Serial.print("Left motor: ");
    Serial.println(speed);
    return;
  }
  
  // Forward - both motors same speed
  if (type == 'F' || type == 'f') {
    int speed = value.toInt();
    speed = constrain(speed, 0, 255);
    setMotors(speed, speed);
    Serial.print("Forward: ");
    Serial.println(speed);
    return;
  }
  
  // Backward - both motors same negative speed
  if (type == 'B' || type == 'b') {
    int speed = value.toInt();
    speed = constrain(speed, 0, 255);
    setMotors(-speed, -speed);
    Serial.print("Backward: ");
    Serial.println(speed);
    return;
  }
  
  // Manual - set both motors independently (M<left>,<right>)
  if (type == 'M' || type == 'm') {
    int commaPos = value.indexOf(',');
    if (commaPos > 0) {
      int leftSpeed = value.substring(0, commaPos).toInt();
      int rightSpeed = value.substring(commaPos + 1).toInt();
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);
      setMotors(leftSpeed, rightSpeed);
      Serial.print("Manual - L:");
      Serial.print(leftSpeed);
      Serial.print(" R:");
      Serial.println(rightSpeed);
      return;
    }
  }
  
  Serial.println("Invalid command!");
}

void setMotors(int left, int right) {
  // Left
  if (left >= 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, left);
  } else {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, -left);
  }

  // Right
  if (right >= 0) {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, right);
  } else {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, -right);
  }
}

void stop() {
  setMotors(0, 0);
  delay(500);
}
