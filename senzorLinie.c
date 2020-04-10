
#define IR_PIN_STANGA (0x04)
#define IR_PIN_CENTRU (0x02)
#define IR_PIN_DREAPTA (0x01)
#define MOTOR_IN1 (0x10)
#define MOTOR_IN2 (0x20)
#define MOTOR_IN3 (0x40)
#define MOTOR_IN4 (0x80)
#define MOTOR_SPEED (95) // Intre [0 - 255]

int l,c,r;

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

void motorInit() {
  noInterrupts(); // Dezactiveaza toate intreruperile

  DDRD |= 0xF0;  // Pin 4,5,6,7 iesire pentru in1, in2, in3, in4 directie motor
  DDRD |= 0x08;  // Pin  3 - PWM output A 
  DDRB |= 0x08;  // Pin 11 - PWM output B
  
  TCCR2A |= (1 << WGM21) | (1 << WGM20);   // Setez fast pwm mode
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1); // Activez ambele canale A=pin 3 si B=pin 11
  TCCR2B |= (1 << CS22) | (1 << CS20);     // Setez prescaler (128)
  OCR2A = MOTOR_SPEED; // Viteza motorului drept
  OCR2B = MOTOR_SPEED; // Viteza motorului stang

  interrupts();
}

void motorStop() {
  PORTD &= ~0xF0;
}

void motorFata(){
  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN3;
}

void motorSpate(){
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN4;
}

void motorStanga(){

  motorStop();
  PORTD |= MOTOR_IN1;
  PORTD |= MOTOR_IN4;
}

void motorDreapta(){
  motorStop();
  PORTD |= MOTOR_IN2;
  PORTD |= MOTOR_IN3;
}



int trackingSensor() 

{

  int irValst = (PIND & IR_PIN_STANGA);
  if(irValst == IR_PIN_STANGA) {
    PORTD &= ~0x04;
    l=01;//------------------------STANGA ALB 1
   // Serial.println(l);
     // alb
  } else {
    PORTB|=0x04;
    l=11; //------------------------STANGA NEGRU 11
    //Serial.println(l);
     // negru

  }
    int irValdr = (PINB & IR_PIN_DREAPTA);
    if(irValdr == IR_PIN_DREAPTA) {
    PORTB &= ~0x01;
    r=02; //------------------------DREAPTA ALB 2
    //Serial.println(r);
     // alb
  } else {
    PORTB|=0x01;
    r=12; //------------------------DREAPTA NEGRU 12
    //Serial.println(r);
     // negru
  }
    int irValc = (PINB & IR_PIN_CENTRU);
    if(irValc == IR_PIN_CENTRU) {
    PORTB &= ~0x02;
    c=03;//------------------------CENTRU ALB 3
   // Serial.println(c);
     // alb
  } else {
    PORTB|=0x02;
    c=13;//------------------------CENTRU NEGRU 13
   // Serial.println(c);
     // negru
  }
   if(c && !l && !r){
     motorDreapta();
     /* Serial.println(c);
     Serial.println(l);
     Serial.println(r); */
  }
  else 
  {
    if(l && !c && !r){
 /* Serial.println(c);
     Serial.println(l);
     Serial.println(r); */
  motorDreapta();
    }
  else {
    if(r && !c && !l){
      /* Serial.println(c);
     Serial.println(l);
     Serial.println(r); */
    motorFata();
    }
    else{
     /*  Serial.println(c);
     Serial.println(l);
     Serial.println(r); */
    motorStop();
  }
}
  }
}

void ir_setup(){
  DDRD&=~0x04; //pin 3 input ir senzor stanga
  DDRB&=~0x03; //pin 8 si 9 input pt centru si dreapta
}
void setup() {
  Serial.begin(9600);
  ir_setup();

}

  
void loop() {
  //int a=2;
  //Serial.println(a);
  trackingSensor();
}
