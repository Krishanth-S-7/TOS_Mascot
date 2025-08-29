// RC Differential Drive with Cytron MDD10A
// Features: Deadzone, Motor Trim, Normalized Mixing, Throttle Print
// 
#include <Servo.h>

#define CH5_PIN 13  // Throttle (forward/backward)
#define CH4_PIN 12    // Steering (left/right)
#define CH3_PIN 9
#define CH6_PIN 2

#define PWM1 6
#define DIR1 4
#define PWM2 5
#define DIR2 7

String cmd;
unsigned long myTime;
int ch6Value, ch5Value, ch4Value, ch3Value;
int leftMotorSpeed, rightMotorSpeed;
int throttle, steer;

// Calibration
const int DEADZONE = 30;       // joystick neutral tolerance
const float LEFT_TRIM  = 0.90; // reduce left motor power
const float RIGHT_TRIM = 1.00; // reduce right motor power
Servo myservo;
int servoPin = 3;

void setup() {
  Serial.begin(9600);
  myservo.attach(3);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  Serial.println("RC + Motor Control with Deadzone + Trim + Normalized Mixing");
}

void loop() {
  int start = 0;
  
  // Read RC signals
  
  ch5Value = pulseIn(CH5_PIN, HIGH, 40000);
  Serial.println(ch5Value);
  ch6Value = pulseIn(CH6_PIN,HIGH, 40000);
  ch3Value = pulseIn(CH3_PIN, HIGH, 40000); 
  ch4Value = pulseIn(CH4_PIN, HIGH, 40000);
  Serial.print("CH3: "); Serial.print(ch3Value);
  Serial.print("  CH4: "); Serial.print(ch4Value);
  Serial.print("  CH5: "); Serial.print(ch5Value);
  Serial.print("  CH6: "); Serial.print(ch6Value);
  

  if (ch5Value > 1500) {



    // Failsafe: neutral if no signal
    if (ch3Value == 0) ch3Value = 1500;
    if (ch4Value == 0) ch4Value = 1500;

    // Convert RC to -255..255
    throttle = map(ch3Value, 1000, 2000, -255, 255);
    steer    = map(ch4Value, 1000, 2000, -255, 255);

    // Apply deadzone
    if (abs(throttle) < DEADZONE) throttle = 0;
    if (abs(steer) < DEADZONE) steer = 0;
    if(ch6Value>1600){
     myservo.write(40);
    }
    else{
      myservo.write(0);
    }
    // ---------------------------
    // Normalized Differential Mixing
    // ---------------------------
    

    float left  = constrain(steer + throttle, -255, 255);
    float right = constrain(throttle-steer, -255, 255);

    // // Normalize to prevent clipping
    // float maxVal = max(abs(left), abs(right));
    // if (maxVal > 1.0) {
    //   left  /= maxVal;
    //   right /= maxVal;
    // }

    // // Back to PWM (-255..255) and apply trim
    // leftMotorSpeed  = (int)(left * 255 * LEFT_TRIM);
    // rightMotorSpeed = (int)(right * 255 * RIGHT_TRIM);

    // Drive motors
    left = -left*0.25;
    right = -right*0.25;
    
    setMotor(DIR1, PWM1, left);
    setMotor(DIR2, PWM2, right);

    // Debug print
    Serial.print("CH3: "); Serial.print(ch3Value);
    Serial.print("  CH4: "); Serial.print(ch4Value);
    Serial.print("  CH5: "); Serial.print(ch5Value);

    Serial.print("  Throttle: "); Serial.print(throttle);
    Serial.print("  Left: "); Serial.print(left);
    Serial.print("  Right: "); Serial.println(right);

    delay(20);
  }

  else{
    if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');

      if (cmd[0] == 'F') {  // Forward
        digitalWrite(DIR1, HIGH);
        int speed = cmd.substring(2).toInt();
        analogWrite(PWM1, speed);

        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, speed);

        Serial.println("Moving Forward");
        Serial.println(speed);
      }
      else if (cmd[0] == 'B') {  // Backward
        digitalWrite(DIR1, LOW);
        int speed = cmd.substring(2).toInt();
        analogWrite(PWM1, speed);

        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, speed);

        Serial.println("Moving Backward");
        Serial.println(speed);
      }
      else if (cmd[0] == 'L') {  // Left turn
        digitalWrite(DIR1, LOW);
        int speed = cmd.substring(2).toInt();
        analogWrite(PWM1, speed);

        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, speed);
        Serial.println(speed);

        Serial.println("Turning Left");
      }
      else if (cmd[0] == 'R') {  // Right turn
        digitalWrite(DIR1, HIGH);
        int speed = cmd.substring(2).toInt();
        analogWrite(PWM1, speed);

        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, speed);

        Serial.println("Turning Right");
        Serial.println(speed);

      }
      else if (cmd[0] == 'C') {  // Stop
        stopMotors();
        }

      else if (cmd[0] == 'S'){
        myservo.write(start);
        myservo.write(start + 40);
        myservo.write(start);
        myservo.write(start + 40);
        myservo.write(start);
        }
      else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
      }
    }

  }

  }



// ------------------
// Motor Control
// ------------------

void stopMotors() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  Serial.println("Motors stopped");
}

void setMotor(int dirPin, int pwmPin, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed);
  }
}
