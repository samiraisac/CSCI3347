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
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_motor();
  init_servo();
  point_sensor_at(90.0);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveForward(150);
  delay(500);
  moveBackward(150);
  delay(500);
  turnLeft(150);
  delay(500);
  turnRight(150);
  delay(500);
  stop();
  point_sensor_at(0.0);
  delay(500);
  point_sensor_at(90);
  delay(500);
  point_sensor_at(180);
  delay(1000);
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

double uss_pulse(unsigned int pulse_time) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ECHO_PIN, HIGH);
  delayMicroseconds(pulse_time);
  digitalWrite(TRIG_PIN, LOW);
  
  long echo_time = pulseIn(ECHO_PIN, HIGH);

  return (echo_time * 17) / 1000;
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

bool is_obstacle(float angle, float detection_cutoff_distance) {
  point_sensor_at(angle);
  float distance = uss_pulse(10);
  if distance > detection_cutoff_distance {
    stop();
    is_
  }
}