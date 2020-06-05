#define IR_PIN_STANGA (0x04) //senzor linie
#define IR_PIN_CENTRU (0x02)  //senzor linie
#define IR_PIN_DREAPTA (0x01)  //senzor linie
#define MOTOR_IN1 (0x10)
#define MOTOR_IN2 (0x20)
#define MOTOR_IN3 (0x40)
#define MOTOR_IN4 (0x80)
#define MOTOR_SPEED_DR (110)
#define MOTOR_SPEED_ST (90)// Intre [0 - 255]
#define MOTOR_SPEED_START_DR (120)
#define MOTOR_SPEED_START_ST (100)

int l, c, r; //senzorii linie, l->stanga,c->centru, r->dreapta

void clearRegisters() {
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  TCCR0A = 0;
  TCCR1A = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  // ADMUX = 0;
}

void motorInit() {
  noInterrupts(); // Dezactiveaza toate intreruperile

  DDRD |= 0xF0; // Pin 4,5,6,7 iesire pentru in1, in2, in3, in4 directie motor
  DDRD |= 0x08; // Pin  3 - PWM output A 
  DDRB |= 0x08; // Pin 11 - PWM output B
  //TIMSK0=1;

  TCCR2A |= (1 << WGM21) | (1 << WGM20); // Setez fast pwm mode
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1); // Activez ambele canale A=pin 3 si B=pin 11
  TCCR2B |= (1 << CS22) | (1 << CS20); // Setez prescaler (128)
  OCR2A = MOTOR_SPEED_ST; // Viteza motorului drept
  OCR2B = MOTOR_SPEED_DR; // Viteza motorului stang

  interrupts();
}

void motorStop() {
  PORTD &= ~0xF0;
}

void motorFata() {

  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN3;

}

void motorSpate() {
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN4;
}

void motorStanga() {

  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN4;
}

void motorDreapta() {
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN3;
}

void motorCurba() {
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN3;
}

//Cand senzorul va fi pe culaore neagra, variabila va avea valoarea 1
//Cand senzorul va fi pe culoare alba, variabila va avea valoarea 0
//c-> centru;  r->rigt; l->left

int trackingSensor() {
  int irVal_st = (PIND & IR_PIN_STANGA);
  if (irVal_st == IR_PIN_STANGA) {
    PORTD &= ~0x04;
    l = 0; //------------------------STANGA ALB 1
  } else {
    PORTB |= 0x04;
    l = 1; //------------------------STANGA NEGRU 11
  }
  int irVal_dr = (PINB & IR_PIN_DREAPTA);
  if (irVal_dr == IR_PIN_DREAPTA) {
    PORTB &= ~0x01;
    r = 0; //------------------------DREAPTA ALB 2
  } else {
    PORTB |= 0x01;
    r = 1; //------------------------DREAPTA NEGRU 12
  }
  int irVal_c = (PINB & IR_PIN_CENTRU);
  if (irVal_c == IR_PIN_CENTRU) {
    PORTB &= ~0x02;
    c = 0; //------------------------CENTRU ALB 3
  } else {
    PORTB |= 0x02;
    c = 1; //------------------------CENTRU NEGRU 13
  }

  if (c && !l && !r) {
    motorFata();
  } else {
    if (l && !c && !r) {
      motorStanga();
    } else {
      if (r && !c && !l) {
        motorDreapta();
      } else {
        if (l && c && !r) {
          motorStanga();
        } else {
          if (!l && c && r) {
            motorDreapta();
          } else {
            motorStop();
          }
        }
      }
    }
  }
}

void waitUS(unsigned long microsec) {
  unsigned long time = micros();
  while (micros() - time < microsec) {
    // Asteapta...
  }
}
void waitMS(unsigned long milisec) {
  unsigned long time = millis();
  while (millis() - time < milisec) {
    // Asteapta...
  }
}
void ir_setup() {
  DDRD &= ~0x04; //pin d2 input ir senzor stanga
  DDRB &= ~0x03; //pin 8 si 9 input pt centru si dreapta
}
void setup() {
  // Serial.begin(9600);
  clearRegisters();
  ir_setup();
  motorInit();

}

int robot_oprit = 1;

void loop() {

  motorFata();
  trackingSensor();

  if (robot_oprit) {
    OCR2B = MOTOR_SPEED_START_DR; // Viteza motorului drept start
    OCR2A = MOTOR_SPEED_START_ST; // Viteza motorului stang start
    waitMS(100); 
    OCR2B = MOTOR_SPEED_DR; // Viteza motorului drept
    OCR2A = MOTOR_SPEED_ST; //Viteza motorului stang
    trackingSensor();
    robot_oprit = 0;
  }
}
