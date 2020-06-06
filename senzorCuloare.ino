//line sensor pins
#define IR_PIN_STANGA (0x04)
#define IR_PIN_CENTRU (0x02)
#define IR_PIN_DREAPTA (0x01)

//motor pins
#define MOTOR_IN1 (0x10)
#define MOTOR_IN2 (0x20)
#define MOTOR_IN3 (0x40)
#define MOTOR_IN4 (0x80)
#define MOTOR_SPEED_DR (90)
#define MOTOR_SPEED_ST (80)// Intre [0 - 255]
#define MOTOR_SPEED_START_DR (130)
#define MOTOR_SPEED_START_ST (130)

#define TRIGGER_VALUE (0x10)
#define ECHO_VALUE (0x20)
#define DIST_INF (9999)

#define LUNGIME_INTERSECTIE 1000
long timer1_ovf = 0;
int culoare = 0;
int l, c, r; //line sensors variables
int val = 0; //value read from photoresistor
int state = 0;
int robot_oprit = 1;

//microseconds delay function
void waitUS(unsigned long microsec) {
  unsigned long time = micros();
  while (micros() - time < microsec) {
    // Asteapta...
  }
}

ISR(TIMER1_OVF_vect) {
  timer1_ovf++;
}

//timer function
void timer1() {
  noInterrupts();
  DDRB |= TRIGGER_VALUE;
  DDRB &= ~ECHO_VALUE;
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << TOIE1);
  interrupts();
}

//initialize motors
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

//stop motors
void motorStop() {
  PORTD &= ~0xF0;
}

//start front motor
void motorFata() {
  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN3;
}

//start back motor
void motorSpate() {
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN4;
}

//start left motor
void motorStanga() {

  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN4;
}

//start right motor
void motorDreapta() {
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN3;
}

void motorStart() {
  motorFata();
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
//clear registers function
void clearRegisters() {
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  // ADMUX = 0;
}

//line following function
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

//setup line sensors
void ir_setup() {
  DDRD &= ~0x04; //pin 3 input ir senzor stanga
  DDRB &= ~0x03; //pin 8 si 9 input pt centru si dreapta
}

//initialize adc
void adc_init() {
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  //system clock frequency and the input clock to the ADC- 128
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //ADC start conversion
  ADMUX |= (1 << REFS0);
}

//Reads analog input and returns the value read from ADC
uint16_t read_adc(uint8_t channel) {
  ADMUX &= 0xF0; //set input AO to A5
  ADMUX |= channel; //select chanel AO to A5
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC))
  ; //wait wile adc conversion are not updated
  return ADC; //read and return voltage
}

//read and return the photoresistor's value
float read_value(uint8_t channel) {
  int voltage = 0;
  int value = 0;
  float reading;
  reading = read_adc(channel);
  voltage = reading * 5.0;
  value = voltage / 10;

  return value;
}

void init_USART() {
  //set baud rate
  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;

  UBRR0L = B01100111;

  UCSR0B |= (1 << TXEN0) | (1 << RXEN0); //enable receiver and transmitter
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); //set frame format
  sei();
}

void USART_Transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)))
  ; //wait for empty transmit buffer
  UDR0 = data; //put the data into buffer and send it
}

//check color
void checkColor(int value) {
  int culoare = 0;
  if (value > 21) //culoarea alba
  {
    culoare = 1; //------>alb
  } else if (value > 19 && value < 21) //culoarea galbena
  {
    culoare = 2; //----->galben
  } else if (value > 19 && value < 21) //culoarea albastra
  {
    culoare = 3; //----->albastru
  } else if (value > 17 && value < 19) //culoare rosie
  {
    culoare = 4; //------>rosie
  }
}

//ultrasonic sensor function
float sensor_HCSR04() {
  float timp, distanta;

  TCNT1 = 0;
  PORTB &= ~TRIGGER_VALUE; //sterge trigger
  waitUS(2);
  PORTB |= TRIGGER_VALUE; // activare semnal trigger 1 logic
  waitUS(10);
  PORTB &= ~TRIGGER_VALUE; // sterge trigger

  TCNT1 = 0;
  timer1_ovf = 0;
  while (!(PINB & ECHO_VALUE)); // Asteapt dupa ECHO pe 1
  while (PINB & ECHO_VALUE); // Cat timp ECHO e pe 1, timer1 va numara

  timp = (((timer1_ovf * 65535) + TCNT1) * 0.0625 f); // Timpul  in secunde
  distanta = ((timp * 0.0343 f) / 2) - 8;
  return distanta;
  //if(distanta > 1 && distanta < 40) {
  //return distanta;
  //} else {
  //return DIST_INF;
  //}
}

void waitMS(unsigned long milisec) {
  unsigned long time = millis();
  while (millis() - time < milisec) {
    // Asteapta...
  }
}
void setup() {
  clearRegisters();
  //Serial.begin(9600);
  ir_setup();
  init_USART();
  adc_init();
  timer1();
  r = 0;
  l = 0;
  c = 1;
  culoare = 0;
  waitMS(3000);
}

void loop() {

  val = read_value(0000);
  checkColor(val);
  if (state == 0) {
    if (culoare == 1) {
      state = 1;
    } else {
      state = 2;
    }
  } else if (state == 1) {
    motorStart();
    trackingSensor();
    if (culoare != 1) {
      state = 2;
    } else {
      state = 1;
    }
  } else if (state == 2) {
    if (culoare == 2 || culoare == 3 || culoare == 4) {
      state = 3;
    } else {
      state = 1;
    }
  } else if (state == 3) {
    if (culoare == 2) {
      motorDreapta();
      waitMS(300);
      motorFata();
      trackingSensor();
    } else if (culoare == 3) {
      motorFata();
      waitMS(LUNGIME_INTERSECTIE);
      trackingSensor();
    } else if (culoare == 4) {
      waitMS(3000);
      motorFata();
      waitMS(LUNGIME_INTERSECTIE);
      trackingSensor();
    } else {
      state = 1;
    }
  } else {
    motorStop();
  }

}
