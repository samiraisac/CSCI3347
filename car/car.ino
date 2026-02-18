#include <Servo.h>

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define ECHO_PIN A4
#define TRIG_PIN A5
#define SERVO_PWM_PIN 3

static Servo servo;

void setup() {
  Serial.begin(9600);
  init_motor();
  init_servo();
  point_sensor_at(90.0);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  if (!isObstacle(90)) {
    moveForward(150);
    delay(100);
    return;
  }

  stop();
  bool leftBlocked = isObstacle(150);
  bool rightBlocked = isObstacle(30);
    
  if (!leftBlocked) {
    turnLeft(150);
    delay(400);
  } 
  else if (!rightBlocked) {
    turnRight(150);
    delay(400);
  }
  else {
    turnLeft(150);
    delay(800); // 180 turn
  }
  stop();
  delay(200);
}

void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void turnRight(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

double getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  return duration * 0.0343 / 2.0;

}

void init_motor() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(IN1, LOW);
  analogWrite(IN2, LOW);
  analogWrite(IN3, LOW);
  analogWrite(IN4, LOW);
}

void init_servo() {
  servo.attach(SERVO_PWM_PIN);
}

//angles go from left to right (0, 180), servo may need to be repositioned on the car
void point_sensor_at(float angle) {
  servo.write(angle);
}

bool isObstacle(float angle) {

  point_sensor_at(angle);
  delay(250);
  
  double distance = getDistance();

  if (distance <= 15.0){
    return true;
  }else {
    return false;
  }
}
