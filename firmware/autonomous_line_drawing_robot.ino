#include <Servo.h>

Servo S1;
Servo S2;
Servo S3;
//Servo pins
const byte S1pwm = 26;
const byte S2pwm = 30;
const byte S3pwm = 36;

int i = 0;
int j = 0;
int turn = 0;

int moveServo(Servo servo, int refPulseWidth, float servoRadAngle){
  const float Pi = 3.142;
  float minPulseWidth = float(refPulseWidth)-1000.0;
  int cmdSignal = (servoRadAngle+(Pi/2))*(2000.0/Pi)+minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}
// MOTOR DRIVER 1 Pins
const int PWMA1 = 3;  // PWM control for top left motor
const int AIN2 = 4;   // Direction control for top left motor
const int AIN1 = 5;   // Direction control for top left motor
const int STBY1 = 6;  // Standby control for motor driver 1
const int PWMB1 = 9;  // PWM control for top right motor
const int BIN2 = 8;   // Direction control for top right motor
const int BIN1 = 7;   // Direction control for top right motor

// MOTOR DRIVER 2 Pins
const int PWMA2 = 12; // PWM control for right back motor
const int A2IN2 = 50; // Direction control for right back motor
const int A2IN1 = 48; // Direction control for right back motor
const int STBY2 = 46; // Standby control for motor driver 2
const int PWMB2 = 13; // PWM control for left back motor
const int B2IN2 = 42; // Direction control for left back motor
const int B2IN1 = 44; // Direction control for left back motor

// Ultra Sonic Sensor Pins
const int pinTrig = 2;
const int pinEcho = 22;

// Safe distances
const float CHECKPOINT_1 = 16.50; // First stop at 10cm
const float CHECKPOINT_2 = 19.0;  // Stop at 15cm for drawing
const float CHECKPOINT_3 = 14.0; // Third stop at 13cm
const float CHECKPOINT_4 = 16.5; // Fourth stop at 15cm
const float CHECKPOINT_5 = 18.0; // Fifth stop at 18cm (no rotation)
const float STOP_THRESHOLD = 0.5;  // Allowable fluctuation range

void setup() {
    pinMode(STBY1, OUTPUT);
    pinMode(PWMA1, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY2, OUTPUT);
    pinMode(PWMA2, OUTPUT);
    pinMode(A2IN1, OUTPUT);
    pinMode(A2IN2, OUTPUT);
    pinMode(PWMB2, OUTPUT);
    pinMode(B2IN1, OUTPUT);
    pinMode(B2IN2, OUTPUT);

    digitalWrite(STBY1, HIGH); // Enable motor driver 1
    digitalWrite(STBY2, HIGH); // Enable motor driver 2

    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);

    ///////
    ///////
    S1.attach(S1pwm);  
    S2.attach(S2pwm);  
    S3.attach(S3pwm);

    moveServo(S1, 1438, 1.2);
    moveServo(S2, 1422, 0); 
    moveServo(S3, 1434, 0);

}

// Function to read stable distance using median filtering
float getFilteredDistance() {
    float readings[5]; 
    for (int i = 0; i < 5; i++) {
        digitalWrite(pinTrig, LOW);
        delayMicroseconds(2);
        digitalWrite(pinTrig, HIGH);
        delayMicroseconds(10);
        digitalWrite(pinTrig, LOW);
        long duration = pulseIn(pinEcho, HIGH);
        readings[i] = (duration * 0.034) / 2;  
        delay(10);
    }
    
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (readings[i] > readings[j]) {
                float temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }
    return readings[2]; 
}
float S2array[180] = {
    1.093, 1.103, 1.112, 1.121, 1.129, 1.137, 1.145, 1.153, 1.160, 1.167,
    1.174, 1.180, 1.186, 1.193, 1.198, 1.204, 1.209, 1.215, 1.220, 1.225,
    1.229, 1.234, 1.238, 1.243, 1.247, 1.251, 1.254, 1.258, 1.262, 1.265,
    1.268, 1.272, 1.275, 1.278, 1.281, 1.283, 1.286, 1.288, 1.291, 1.293,
    1.295, 1.297, 1.299, 1.301, 1.303, 1.304, 1.306, 1.307, 1.309, 1.310,
    1.311, 1.312, 1.313, 1.314, 1.315, 1.316, 1.316, 1.317, 1.317, 1.318,
    1.318, 1.318, 1.318, 1.318, 1.318, 1.318, 1.318, 1.317, 1.317, 1.316,
    1.316, 1.315, 1.314, 1.313, 1.313, 1.312, 1.310, 1.309, 1.308, 1.307,
    1.305, 1.304, 1.302, 1.301, 1.299, 1.297, 1.295, 1.293, 1.291, 1.289,
    1.287, 1.285, 1.282, 1.280, 1.277, 1.275, 1.272, 1.269, 1.266, 1.263,
    1.260, 1.257, 1.254, 1.251, 1.248, 1.244, 1.241, 1.237, 1.234, 1.230,
    1.226, 1.223, 1.219, 1.215, 1.211, 1.207, 1.202, 1.198, 1.194, 1.189,
    1.185, 1.180, 1.176, 1.171, 1.166, 1.162, 1.157, 1.152, 1.147, 1.142,
    1.136, 1.131, 1.126, 1.120, 1.115, 1.109, 1.104, 1.098, 1.092, 1.087,
    1.081, -1.087, -1.092, -1.098, -1.104, -1.109, -1.115, -1.120, -1.126, -1.131,
    -1.136, -1.142, -1.147, -1.152, -1.157, -1.162, -1.166, -1.171, -1.176, -1.180,
    -1.185, -1.189, -1.194, -1.198, -1.202, -1.207, -1.211, -1.215, -1.219, -1.223,
    -1.226, -1.230, -1.234, -1.237, -1.241, -1.244, -1.248, -1.251, -1.254, -1.257, 0
};

float S3array[180] = {
    -0.615, -0.635, -0.654, -0.673, -0.691, -0.708, -0.726, -0.742, -0.758, -0.774,
    -0.790, -0.805, -0.819, -0.834, -0.848, -0.862, -0.875, -0.889, -0.902, -0.915,
    -0.927, -0.940, -0.952, -0.964, -0.976, -0.987, -0.999, -1.010, -1.021, -1.032,
    -1.042, -1.053, -1.063, -1.074, -1.084, -1.094, -1.103, -1.113, -1.122, -1.132,
    -1.141, -1.150, -1.159, -1.168, -1.177, -1.185, -1.194, -1.202, -1.210, -1.219,
    -1.227, -1.234, -1.242, -1.250, -1.258, -1.265, -1.272, -1.280, -1.287, -1.294,
    -1.301, -1.308, -1.314, -1.321, -1.328, -1.334, -1.341, -1.347, -1.353, -1.359,
    -1.365, -1.371, -1.377, -1.383, -1.388, -1.394, -1.399, -1.405, -1.410, -1.415,
    -1.420, -1.425, -1.430, -1.435, -1.440, -1.444, -1.449, -1.453, -1.458, -1.462,
    -1.466, -1.471, -1.475, -1.479, -1.483, -1.486, -1.490, -1.494, -1.497, -1.501,
    -1.504, -1.507, -1.511, -1.514, -1.517, -1.520, -1.523, -1.525, -1.528, -1.531,
    -1.533, -1.536, -1.538, -1.540, -1.543, -1.545, -1.547, -1.549, -1.551, -1.552,
    -1.554, -1.556, -1.557, -1.559, -1.560, -1.561, -1.563, -1.564, -1.565, -1.566,
    -1.567, -1.567, -1.568, -1.569, -1.569, -1.570, -1.570, -1.570, -1.571, -1.571,
    -1.571, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570,
    -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570, -1.570
};

// Function to control individual motors
void controlMotor(int pin1, int pin2, int pinPWM, int speed) {
    if (speed > 0) {  
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else if (speed < 0) { 
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        speed = -speed;
    } else { 
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
    analogWrite(pinPWM, speed);
}

// Move forward with drift correction
void moveForward(int baseSpeed) {
    int leftSpeed = baseSpeed;  // Slightly boost left motors
    int rightSpeed = baseSpeed +7; // Slightly reduce right motors

    controlMotor(AIN2, AIN1, PWMA1, leftSpeed);   // Top left
    controlMotor(BIN1, BIN2, PWMB1, rightSpeed);  // Top right
    controlMotor(A2IN1, A2IN2, PWMA2, rightSpeed); // Back right
    controlMotor(B2IN2, B2IN1, PWMB2, leftSpeed);  // Back left
}

// Rotate counterclockwise
void rotateCCW(int speed, int duration) {
    controlMotor(AIN1, AIN2, PWMA1, speed);
    controlMotor(BIN1, BIN2, PWMB1, speed);
    controlMotor(A2IN1, A2IN2, PWMA2, speed);
    controlMotor(B2IN1, B2IN2, PWMB2, speed);
    delay(duration);
    stopMotors();
}

// Stop all motors
void stopMotors() {
    controlMotor(AIN2, AIN1, PWMA1, 0);
    controlMotor(BIN1, BIN2, PWMB1, 0);
    controlMotor(A2IN1, A2IN2, PWMA2, 0);
    controlMotor(B2IN2, B2IN1, PWMB2, 0);
}

// Move to specific checkpoint
void moveToCheckpoint(float targetDistance) {
    while (getFilteredDistance() > targetDistance + STOP_THRESHOLD) {
        moveForward(100);
    }
    stopMotors();
    delay(2000);
}

// Main Loop
void loop() {
    delay(3000);
    rotateCCW(80, 3000);
    delay(2000);
    moveToCheckpoint(CHECKPOINT_1);
    rotateCCW(80, 1550);
    delay(2000);

    moveToCheckpoint(CHECKPOINT_2);
    delay(2000);
      if (turn == 0) {
    for (i = 0; i < 180; i++) {
      moveServo(S2, 1422, S2array[i]);
      moveServo(S3, 1434, S3array[i]);
      if (i >= 179) {
        turn += 1;
      }
      delay(50);
    }
  }
    moveToCheckpoint(CHECKPOINT_3);
    //moveForward(20);
    //delay(500);
    rotateCCW(80, 1650);
    delay(1000);
    moveServo(S2, 1422, 0);
    moveServo(S3, 1434, 0);
    delay(2000);

    moveToCheckpoint(CHECKPOINT_4);
    rotateCCW(80, 1650);
    delay(2000);

    moveToCheckpoint(CHECKPOINT_5);
    delay(3000);
    moveServo(S2, 1422, 0.7);
    moveServo(S3, 1434, -1.57);
    delay(2000);
    moveServo(S1, 1438, 1.57);
    while (true);
    
}
