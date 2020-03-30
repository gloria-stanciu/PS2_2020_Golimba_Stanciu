#define MOTOR_IN1 (0x10)
#define MOTOR_IN2 (0x20)
#define MOTOR_IN3 (0x40)
#define MOTOR_IN4 (0x80)
#define MOTOR_SPEED (95) // Intre [0 - 255]

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
