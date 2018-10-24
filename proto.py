from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from controls import Axis, Remote, Motor, Control
import pygame

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
pressed = False;

try:
  connection = SerialManager();
  a = ArduinoApi(connection=connection)
except:
  print("Failed to connect to Arduino")

# Setup
pygame.init()
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Detected ", joystick.get_name())
else:
    raise ValueError("No joystick detected!")

right = Motor(a, ENA, IN1, IN2, 80)
left = Motor(a, ENB, IN3, IN4, 70)
x = Axis(0, 1.000, 0.000, 0, -1.000, 0, 0.1)
y = Axis(5, 1.000, -1.000, 2, 1.000, -1.000, 0.1)
ds3 = Remote(joystick, x, y)
ctl = Control(a, right, left, ds3)

a.pinMode(BTN, a.INPUT)
a.pinMode(LED, a.OUTPUT)

# Loop
while True:
    print(a.analogRead(0))
    print('h')
    mag = 200
    if pressed: a.analogWrite(LED,mag)
    else: a.analogWrite(LED,0)
    
    if a.digitalRead(BTN): pressed = not pressed
    while a.digitalRead(BTN): print("btn waiting\n")
    sleep(0.02)

    ctl.listen()
    print("listened")
    sleep(0.02)



