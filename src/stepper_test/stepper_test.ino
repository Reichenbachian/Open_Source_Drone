#define esc_1 8
#define esc_2 9
#define esc_3 10
#define esc_4 11

#include <Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int throttlePin = 0;
int throttle = 0;

void setup()
{
  Serial.begin(9600);
  esc1.attach(esc_1);
  esc2.attach(esc_2);
  esc3.attach(esc_3);
  esc4.attach(esc_4);
}

void loop()
{
  int tmp = Serial.parseInt();
  if (tmp != 0) throttle = tmp;
  Serial.println(throttle);
  delay(100);
//  throttle = map(throttle, 0, 1023, 0, 179);
  esc1.write(throttle);
  esc2.write(throttle);
  esc3.write(throttle);
  esc4.write(throttle);
}

