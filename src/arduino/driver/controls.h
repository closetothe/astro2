class Motor{
  private:
  int en, in1, in2;
  int MOTORLOW = LOW;
  int MOTORHIGH = HIGH;  
  int led;
  
  public:
  int pwm;
  int deadband = 70;
  int isStopped = true;

  
  Motor(){}
  Motor(int en, int in1, int in2, int db, int led){
    this->en = en;
    this->in1 = in1;
    this->in2 = in2;
    this->deadband = db;
    this->led = led;
    pinMode(this->en, OUTPUT);
    pinMode(this->in1, OUTPUT);
    pinMode(this->in2, OUTPUT);    
  }
  Motor(int en, int in1, int in2, int db, int led, int polarity){
    if (polarity) {
      this->MOTORLOW = HIGH;
      this->MOTORHIGH = LOW;
    }
    Motor(en, in1, in2, db, led);
  }

  void init(){
    digitalWrite(this->in1, MOTORLOW);
    digitalWrite(this->in2, MOTORHIGH); 
    analogWrite(this->en, 0);
  }

  void forward(){
    digitalWrite(this->in1, MOTORLOW);
    digitalWrite(this->in2, MOTORHIGH);    
  }

  void reverse(){
    digitalWrite(this->in1, MOTORHIGH);
    digitalWrite(this->in2, MOTORLOW);     
  }

  void stop(){
    setSpeed(0);
  }

  void setSpeed(int pwm){
    if(pwm > deadband) {
      this->pwm = pwm;
      isStopped = false; 
      digitalWrite(led, HIGH);
    }
    else {
      this->pwm = 0;
      forward();
      isStopped = true;
      digitalWrite(led, LOW); 
    }
    analogWrite(this->en, this->pwm);  
  }
};
