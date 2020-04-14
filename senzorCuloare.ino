//adc initialization
void adc_init()
{
  ADCSRA|=((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
  //system clock frequency and the input clock to the ADC- 128
  ADCSRA|=(1<<ADEN); //enable ADC
  ADCSRA|=(1<<ADSC); //ADC start conversion
  ADMUX|=(1<<REFS0);
}

//Reads analog input and returns the value read from ADC 
uint16_t read_adc(uint8_t channel) 
{
  ADMUX &= 0xF0; //set input AO to A5
  ADMUX|=channel; //select chanel AO to A5
  ADCSRA|=(1<<ADSC); //start conversion
  while(ADCSRA & (1<<ADSC)); //wait wile adc conversion are not updated
  return ADC; //read and return voltage
}

//read and return value of photoresistor
float read_value(uint8_t channel) 
{
  int voltage = 0;
  int value = 0;
  float reading;
  reading = read_adc(channel);
  voltage = reading*5.0;
  value = voltage/10;

  return value;
}

void init_USART()
{
  //set baud rate
  UCSR0A=0;
  UCSR0B=0;
  UCSR0C=0;

  UBRR0L=B01100111;

  UCSR0B|=(1<<TXEN0)|(1<<RXEN0); //enable receiver and transmitter
  UCSR0C=(1<<UCSZ00)|(1<<UCSZ01); //set frame format
  sei();
}

void USART_Transmit(unsigned char data)
{
  while (!(UCSR0A & (1<<UDRE0))); //wait for empty transmit buffer
  UDR0 = data; //put the data into buffer and send it
}

int checkColor(int value){
  if (value < 150){
    PORTD = B00000100;

    return 1;
  }
  else if (value > 150 && value < 250){
    PORTD = B00010100;

    return 2;
  }
  else if(value>250){
    PORTD = B00001000;
    return 3;
  }
}

int val=0; //valoarea citita de la fotorezistor
int color=0; //culoarea returnata

int main()
{
  DDRD = B00011100;
  //red:   B00000100
  //blue:  B00001000
  //green: B00010000
  char buf[100]; //buffer pentru afisare temperatura
 
  init_USART();
  adc_init();

  while(1)
  {

    memset(buf,0,sizeof(buf));
    sprintf(buf,"Valoarea este %d\n", val);
    val=read_value(0000);
    for(int i=0;i<strlen(buf);i++)
    {
      USART_Transmit(buf[i]);
    }
    
    color = checkColor(val);

    
    //if(color == 1) //if color is red
    //{
      //merge putin in fata
      //asteapta 3 secunde
    //}
    //if(color == 2) //if color is yellow
//{
     //vireaza la dreapta, ungi de 90 de grade 
    //}
    //if(color == 3) //if color is blue
    //{
      //robotul traverseaza intersectia fara sa se opreasca
    //}
  }
}
