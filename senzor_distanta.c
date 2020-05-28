#define TRIGGER_VALUE (0x10)
#define ECHO_VALUE (0x20)
#define DIST_INF (9999)



float sensor_HCSR04(){
  float timp, distanta;

  TCNT1 = 0;
  PORTB &= ~TRIGGER_VALUE;   //sterge trigger
  waitUS(2);
  PORTB |= TRIGGER_VALUE;   // activare semnal trigger 1 logic
  waitUS(10);
  PORTB &= ~TRIGGER_VALUE; // sterge trigger

  TCNT1 = 0;
  timer1_ovf = 0;

  while(!(PINB & ECHO_VALUE)); // Asteapt dupa ECHO pe 1
  while(PINB & ECHO_VALUE); // Cat timp ECHO e pe 1, timer1 va numara

  timp = (((timer1_ovf * 65535) + TCNT1) * 0.0625f); // Timpul  in secunde 
  distanta = ((timp * 0.0343f) / 2) - 8;

  if(distanta > 1 && distanta < 15) {
    return distanta;
  } else {
    return DIST_INF;
  }
}