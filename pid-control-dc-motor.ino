
float kp = 0.1;
float ki = 0.00015 ;//0.00005 ;//0.00015 ;
float kd = 0;

unsigned long t;
unsigned long t_prev = 0;

const byte interruptPinA = 2;
const byte interruptPinB = 3;
volatile long EncoderCount = 0;
const byte PWMPin = 6;
const byte DirPin1 = 7;
const byte DirPin2 = 8;

volatile unsigned long count = 0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d;
float Theta_prev = 0;
int dt;



float e, e_prev = 0, inte, inte_prev = 0;

float dedt;
float u;


void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }
}



void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }
}






void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);    //Set speed
  if (dir == 1) {              //if dir = 1, rotate one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {       //if dir = -1, rotate ohter way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {                    //if dir = 0,do not rotate
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }


}


void setup() {
  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void loop() {
  if (count > count_prev) {
    t = millis();

    dt = (t - t_prev);
    RPM_d = 0;

    e = RPM_d - RPM;

    inte = inte_prev + (dt * (e + e_prev) / 2); //Trapezoidal rule    inte+=inte*dt;

    dedt = (e - e_prev) / dt;

    u = kp * e + kd * dedt + ki * inte;


    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }

    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
    
    setMotor(dir, pwr, PWMPin, DirPin1, DirPin2);

    //Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
    Serial.println("rpm: ");
    Serial.println(RPM);
   

  }
}

ISR(TIMER1_COMPA_vect) {
  count++;
  Theta = EncoderCount / 64.0;
  RPM = (Theta - Theta_prev) / (50 / 1000.0) * 60;
  Theta_prev = Theta;
  // Serial.print(count ); Serial.print(" \n");
}
