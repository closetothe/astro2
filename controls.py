from time import sleep

def amap(x,in_min,in_max,out_min,out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

XLIMIT = 180

class Axis:
    # Axis index (in pyjoy), with max and min values
    def __init__(self, p, pmax, pmin, n, nmax, nmin, tol)
        self.p = p
        self.pmax = pmax - tol
        self.pmin = pmin + tol
        self.n = n
        self.nmax = nmax - tol
        self.nmin = nmin + tol

class Remote:

    def __init__(self, joystick, xaxis, yaxis):
        self.xaxis = xaxix
        self.yaxis = yaxis

    def getx(self):
        xpos = joystick.get_axis(self.xaxis.p)
        xneg = joystick.get_axis(self.xaxis.p)
        if xpos > self.xaxis.pmin:
            return amap(xpos, self.xaxis.pmin, self.xaxis.pmax, 0, XLIMIT)
        elif xneg > self.xaxis.nmin:
            return (-1)*(amap(xneg, self.xaxis.nmin, self.xaxis.nmax, 0, XLIMIT))
        else return 0

    def gety(self):
        ypos = joystick.get_axis(self.yaxis.p)
        yneg = joystick.get_axis(self.yaxis.p)
        if ypos > self.yaxis.pmin:
            return amap(ypos, self.yaxis.pmin, self.yaxis.pmax, 0, 255)
        elif yneg > self.yaxis.nmin:
            return (-1)*(amap(yneg, self.yaxis.nmin, self.yaxis.nmax, 0, 255))
        else return 0





# For an Arduino potentiometer
class Pot:
    def __init__(self, arduino, x_pin, y_pin, tol):
        a = arduino
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
    def __init__(self, arduino, en, in1, in2, db):
        print("Motor instantiated")
        a = arduino
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

# Simple control system
class Control:
    def __init__(self, arduino, left, right, pot):
        a = arduino
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
            l.setSpeed(l.pwm + dir*x)
            r.setSpeed(r.pwm - dir*x)

        if lpw < 0: l.setSpeed(0)
        if lpw > XLIMIT: l.setSpeed(XLIMIT)
        if rpw < 0: r.setSpeed(0)
        if rpw > XLIMIT: r.setSpeed(XLIMIT)
