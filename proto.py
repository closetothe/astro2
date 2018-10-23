# from nanpy import (ArduinoApi, SerialManager)
# from time import sleep
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
#POTX = a.A0
#POTY = a.A1

pressed = False;

# try:
#   connection = SerialManager();
#   a = ArduinoApi(connection=connection)
# except:
#   print("Failed to connect to Arduino")


m1 = Motor(ENA, IN1, IN2, 30)
m2 = Motor(ENB, IN3, IN4, 30)
ctl = Control








