from nanpy import (ArduinoApi, SerialManager)
from time import sleep


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

POTX = 0
POTY = 1

def amap(x,in_min,in_max,out_min,out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

class Pot:
    def __init__(self, x_pin, y_pin, tol):
        self.__x_pin = x_pin
        self.__y_pin = y_pin
        self.tol = 50
        self.center = 512
        a.pinMode(self.__x_pin, a.INPUT)
        a.pinMode(self.__y_pin, a.INPUT)
    
    def x(self):
        print(a.analogRead(self.__x_pin))
        return a.analogRead(self.__x_pin)
       

    def y(self):
        return a.analogRead(self.__y_pin)
        

class Motor:
    def __init__(self, en, in1, in2, db):
        print("Motor instantiated")
        self.__en = en
        self.__in1 = in1
        self.__in2 = in2
        self.deadband = db
        self.pwm = 0
        self.isStopped = True
        a.digitalWrite(self.__in1, a.LOW)
        a.digitalWrite(self.__in2, a.HIGH) 
        a.analogWrite(self.__en, 0)
        sleep(0.01)    
        a.pinMode(self.__en, a.OUTPUT)
        a.pinMode(self.__in1, a.OUTPUT)
        a.pinMode(self.__in2, a.OUTPUT)

    def forward(self):
        print("fw")
        a.digitalWrite(self.__in1, a.LOW)
        a.digitalWrite(self.__in2, a.HIGH)    
  

    def reverse(self):
        print("bw")
        a.digitalWrite(self.__in1, a.HIGH)
        a.digitalWrite(self.__in2, a.LOW)     
  
    def setSpeed(self, pwm):
        if pwm > self.deadband:
            self.pwm = pwm
            self.isStopped = False
        else :
            self.pwm = pwm
            self.isStopped = True
            self.forward()
        a.analogWrite(self.__en, self.pwm)

    def stop(self):
        self.setSpeed(0)

class Control:
    def __init__(self, left, right, pot, kx):
        self.__kx = kx
        self.__left = left
        self.__right = right
        self.__pot = pot

    def listen(self):
        x = self.__pot.x()
        print(x)
        print("found x")
        y = self.__pot.y()
        print("found y")
        l = self.__left
        r = self.__right
        tol = self.__pot.tol
        center = self.__pot.center

        # Y Axis
        if y < (center - tol):
            pwm_y = amap(y, (center - tol), 0, 0, 255)
            l.reverse()
            r.reverse()
            l.setSpeed(pwm_y)
            r.setSpeed(pwm_y)

        elif y > (center + tol):
            pwm_y = amap(y, (center + tol), 1023, 0, 255);
            l.forward()
            r.forward()
            l.setSpeed(pwm_y)
            r.setSpeed(pwm_y)
        else:
            l.stop()
            r.stop()
        print("end of y axis")

        if x < (center - tol):
            # left turn
            self.turn(x, -1, 0)

        elif x > (center + tol):
            # right turn
            self.turn(x, 1, 1023)

    def turn(self, amt, dir, n):
        k = self.__kx
        l = self.__left
        r = self.__right
        lpw = l.pwm
        rpw = r.pwm
        tol = self.__pot.tol
        ctr = self.__pot.center
        x = amap(amt, (ctr + tol), n, 0, 180);

        if l.isStopped and r.isStopped:
            # zero radius turn
            if dir < 0:
                l.reverse()
                r.forward()
            else:
                l.forward()
                r.reverse()
            l.setSpeed(x)
            r.setSpeed(x)
        else:
            l.setSpeed(lpw + k*dir*x)
            r.setSpeed(rpw - k*dir*x)

        if lpw < 0: l.setSpeed(0)
        if lpw > 255: l.setSpeed(255)
        if rpw < 0: r.setSpeed(0)
        if rpw > 255: r.setSpeed(255)



# Setup
right = Motor(ENA, IN1, IN2, 80)
left = Motor(ENB, IN3, IN4, 70)
pot = Pot(POTX, POTY, 20)
ctl = Control(right, left, pot, 1)

a.pinMode(BTN, a.INPUT)
a.pinMode(LED, a.OUTPUT)
# try:
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

#except:
#    print("Event loop failed.")



