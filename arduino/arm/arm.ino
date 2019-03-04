#include <Servo.h>

#define ENH 12
#define READH A0

Servo servo;

void setup() {
  Serial.begin(9600);
  pinMode(ENH, OUTPUT);
  servo.attach(ENH);
}
int i = 0;
void loop() {
  int pos = 0;
  pos = analogRead(READH);
  int cmd = 60;
  servo.write(67);
  Serial.print("CMD: ");
  Serial.print(cmd);
  Serial.print(" Position: ");
  Serial.println(pos);
  delay(50);
}
