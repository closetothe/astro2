from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from controls import Pot
from controls import Motor
from controls import Control

# H-bridge inputs
# Motor 1
ENA = 5
IN1 = 6
IN2 = 7
# Motor 2
ENB = 8
IN3 = 2
IN4 = 3
# Digital i/o
BTN = 4
LED = 9
POTX = a.A0
POTY = a.A1

pressed = False;

try:
  connection = SerialManager();
  a = ArduinoApi(connection=connection)
except:
  print("Failed to connect to Arduino")

# Setup
right = Motor(ENA, IN1, IN2, 80)
left = Motor(ENB, IN3, IN4, 70)
pot = Pot(POTX, POTY, 20)
ctl = Control(right, left, pot, 1)

a.pinMode(BTN, a.INPUT)
a.pinMode(LED, a.OUTPUT)

try:
    while True:
        mag = 200
        if pressed: a.analogWrite(LED,mag)
        else: a.analogWrite(LED,0)

        if a.digitalRead(BTN): pressed = !pressed
        while a.digitalRead(BTN):
        sleep(0.02)

        ctl.listen()
        sleep(0.02)

except:
    print("Event loop failed.")






