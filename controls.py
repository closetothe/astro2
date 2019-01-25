from time import sleep

def amap(x,in_min,in_max,out_min,out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

XLIMIT = 220

class Axis:
    # Axis index (in pyjoy), with max and min values
    # Assuming thumbstick for x
    def __init__(self, p, pmax, pmin, n, nmax, nmin, tol):
        self.p = p
        self.pmax = pmax
        self.pmin = pmin
        self.pmint = pmin + tol
        self.n = n
        self.nmax = nmax
        self.nmin = nmin
        self.nmint = nmin + tol

class Remote:

    def __init__(self, joystick, xaxis, yaxis):
        self.joystick = joystick
        self.xaxis = xaxis
        self.yaxis = yaxis

    def getx(self):
        x = self.joystick.get_axis(self.xaxis.p)
        #x = self.joystick.get_axis(self.xaxis.n)
        #print(xpos, " ", xneg)
        if x > self.xaxis.pmint:
            pwm = amap(x, self.xaxis.pmin, self.xaxis.pmax, 0, XLIMIT)
            print(pwm)
            return pwm
        elif abs(x) > self.xaxis.pmint:
            pwm = (-1)*(amap(abs(x), self.xaxis.pmin, self.xaxis.pmax, 0, XLIMIT))
            print(pwm)
            return pwm
        else: return 0

    def gety(self):
        ypos = self.joystick.get_axis(self.yaxis.p)
        yneg = self.joystick.get_axis(self.yaxis.n)
        #print(ypos, " ", yneg)
        if ypos > self.yaxis.pmint:
            pwm = amap(ypos, self.yaxis.pmin, self.yaxis.pmax, 0, 255)
            print(pwm)
            return pwm
        elif yneg > self.yaxis.nmint:
            pwm = (-1)*(amap(yneg, self.yaxis.nmin, self.yaxis.nmax, 0, 255))
            print(pwm)
            return pwm
        else: return 0





# For an Arduino potentiometer
class Pot:
    def __init__(self, a, arduino, x_pin, y_pin, tol):
        self.a = arduino
        self.__x_pin = x_pin
        self.__y_pin = y_pin
        self.tol = 50
        self.center = 512
        self.a.pinMode(self.__x_pin, a.INPUT)
        self.a.pinMode(self.__y_pin, a.INPUT)
    
    def x(self):
        print(self.a.analogRead(self.__x_pin))
        return self.a.analogRead(self.__x_pin)
       

    def y(self):
        return self.a.analogRead(self.__y_pin)
        

class Motor:
    def __init__(self, arduino, en, in1, in2, db):
        print("Motor instantiated")
        self.a = arduino
        a = self.a
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
        #print("fw")
        self.a.digitalWrite(self.__in1, self.a.LOW)
        self.a.digitalWrite(self.__in2, self.a.HIGH)    
  

    def reverse(self):
        #print("bw")
        self.a.digitalWrite(self.__in1, self.a.HIGH)
        self.a.digitalWrite(self.__in2, self.a.LOW)     
  
    def setSpeed(self, pwm):
        if pwm > self.deadband:
            self.pwm = pwm
            self.isStopped = False
        else :
            self.pwm = pwm
            self.isStopped = True
            self.forward()
        self.a.analogWrite(self.__en, self.pwm)

    def stop(self):
        self.setSpeed(0)

# Simple control system
class Control:
    def __init__(self, left, right, pot):
        self._left = left
        self._right = right
        self._pot = pot

    def listen(self):
        # returns pwm value
        y = self._pot.gety()
        x = self._pot.getx()

        l = self._left
        r = self._right

        # Y Axis
        if y > 0 :
            #pwm_y = amap(ypos, self._pot.yaxis.pmin + ytol, 0, 0, 255)
            l.forward()
            r.forward()
            l.setSpeed(abs(y))
            r.setSpeed(abs(y))

        elif y < 0:
            #pwm_y = amap(y, (center + tol), 1023, 0, 255);
            l.reverse()
            r.reverse()
            l.setSpeed(abs(y))
            r.setSpeed(abs(y))
        else:
            l.stop()
            r.stop()
        

        if x < 0:
            # turn left
            self.turn(x, -1)

        elif x > 0:
            # turn right
            self.turn(x, 1)

    def turn(self, x, dir):
        l = self._left
        r = self._right

        if l.isStopped and r.isStopped:
            # zero radius turn
            if dir < 0:
                l.reverse()
                r.forward()
            else:
                l.forward()
                r.reverse()
            l.setSpeed(abs(x))
            r.setSpeed(abs(x))
        else:
            l.setSpeed(l.pwm + dir*abs(x))
            r.setSpeed(r.pwm - dir*abs(x))

        if l.pwm < 0: l.setSpeed(0)
        if l.pwm > XLIMIT: l.setSpeed(XLIMIT)
        if r.pwm < 0: r.setSpeed(0)
        if r.pwm > XLIMIT: r.setSpeed(XLIMIT)
