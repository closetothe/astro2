class Pot:
    def __init__(self, x_pin, y_pin, tol):
        self.__x_pin = x_pin
        self.__y_pin = y_pin
        self.tol = 50
        self.center = 512
        # a.pinMode(self.__x_pin, a.INPUT)
        # a.pinMode(self.__y_pin, a.INPUT)
    
    def x(self):
        #return a.analogRead(self.__x_pin)
        return "x"

    def y(self):
        #return a.analogRead(self.__y_pin)
        return "y"

class Motor:
    def __init__(self, en, in1, in2, db):
        print("Motor instantiated")
        self.__en = en
        self.__in1 = in1
        self.__in2 = in2
        self.deadband = db
        self.pwm = 0
        self.isStopped = True
        # a.digitalWrite(self.__in1, a.LOW);
        # a.digitalWrite(self.__in2, a.HIGH); 
        # a.analogWrite(self.__en, 0);
        # a.delay(10)       
        # a.pinMode(self.__en, a.OUTPUT);
        # a.pinMode(self.__in1, a.OUTPUT);
        # a.pinMode(self.__in2, a.OUTPUT); 

    def forward(self):
        print("fw")
        # a.digitalWrite(self.__in1, a.LOW);
        # a.digitalWrite(self.__in2, a.HIGH);    
  

    def reverse(self):
        print("bw")
        # a.digitalWrite(self.__in1, a.HIGH);
        # a.digitalWrite(self.__in2, a.LOW);     
  

    def stop(self):
        setSpeed(0);
  
    def setSpeed(pwm):
        if pwm > self.deadband:
            self.pwm = pwm
            self.isStopped = False
        else :
            self.pwm = pwm
            self.isStopped = True
            forward()
        a.analogWrite(self.__en, self.pwm)


class Control:
    def __init__(self, left, right, pot, kx):
        self.__kx = kx
        self.__left = left
        self.__right = right
        self.__pot = pot

    def listen(self):
        x = self.__pot.x()
        y = self.__pot.y()
        l = self.__left
        r = self.__right
        tol = self.__pot.tol
        center = self.__pot.center

        # Y Axis
        if y < (center - tol):
            #pwm_y = a.map(y, (center - tol), 0, 0, 255)
            l.reverse()
            r.reverse()
            l.setSpeed(pwm_y)
            r.setSpeed(pwm_y)

        elif y > (center + tol):
            #pwm_y = a.map(y, (center + tol), 1023, 0, 255);
            l.forward()
            r.forward()
            l.setSpeed(pwm_y)
            r.setSpeed(pwm_y)
        else:
            l.stop()
            r.stop()

        if x < (center - tol):
            # left turn
            turn(x, -1, 0)

        elif x > (center + tol):
            # right turn
            turn(x, 1, 1023)

    def turn(amt, dir, n):
        k = self.__kx
        l = self.__left
        r = self.__right
        lpw = l.pwm
        rpw = r.pwm
        tol = self.__pot.tol
        ctr = self.__pot.center
        x = 1   
        #x = a.map(amt, (ctr + tol), n, 0, 180);

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
