#include <Servo.h>
#define PIN_SERVO 10
#define DELAY 350
#define PIN_IR A0
#define ALPHA 0.5
Servo myservo;

float raw_dist;
float dist_cali;

float f(float raw, float a, float b) {
  return 300 / (b - a) * (raw - a) + 100;
}

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1550);
  delay(DELAY);
  Serial.begin(57600);
}


float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

void loop() {
  raw_dist = ir_distance();
  dist_cali = dist_cali*ALPHA + f(raw_dist, 70, 270)*(1-ALPHA);
  Serial.print(raw_dist);
  Serial.print(" ");
  Serial.println(dist_cali);
  
  if (dist_cali > 250) myservo.writeMicroseconds(1350);
  else myservo.writeMicroseconds(1750);
  delay(200);
}
