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
size = [100,100]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Astro Prototype 2.1")
clock = pygame.time.Clock()

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Detected ", joystick.get_name())
else:
    raise ValueError("No joystick detected!")

right = Motor(a, ENA, IN1, IN2, 50)
left = Motor(a, ENB, IN3, IN4, 50)
x = Axis(0, 1.000, 0.000, 0, 0.000, -1.000, 0.4)
y = Axis(5, 1.000, -1.000, 2, 1.000, -1.000, 0.4)
ds3 = Remote(joystick, x, y)
ctl = Control(left, right, ds3)
right.stop()
left.stop()
a.pinMode(BTN, a.INPUT)
a.pinMode(LED, a.OUTPUT)
i = 0
# Loop
while True:
    pygame.event.pump()
    if i < 50:
        right.stop()
        left.stop()
    else:
        ctl.listen()
    pygame.display.flip()
    i = i + 1
    clock.tick(60)


pygame.quit()


