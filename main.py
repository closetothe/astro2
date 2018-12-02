from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from controls import Axis, Remote, Motor, Control
import pygame
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

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
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(640,480))
sleep(0.2)

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
sleep(0.1)

# Loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    pygame.event.pump()
    if joystick.get_button(0):
        os.system("raspistill --width 1920 --height 1080 -o photo.jpg")
        print("Photo saved.")
    if joystick.get_button(9):
        break
    else:
        ctl.listen()
    img = frame.array
 
    if joystick.get_button(1):
        cv2.imshow("Video Feed", img)
        key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    pygame.display.flip()
    clock.tick(60)


pygame.quit()


