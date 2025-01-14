# embedded-project
light detecting car
//Ultrasonic  Trig  RC6 ,, Echo RC7
// SERVO PIN RC2
// PWM  EN MOTORS RC1
// DiRECTION PINS RB0 , RB1 , RB2 , RB3
// LDR RD0 , RD1 , RD2
// Analog LDR RA0
//Start  Buttno RD3
//LEDS Pin RD6, RD7

int tick;
int tick1;
unsigned int sensor_voltage;
unsigned char H_L;
unsigned int angle;
int distance;
int a;
int b;



void initialize();
void interrupt();

void stop_moving();
void move_left();
void move_right();
void move_forward();
void move_backwards();
void adjust_position();
void ENDD();
int dist();


void check_start(){
 if(PORTD & 0B00001000){
       b++;
     }
     if(b%2==0){
     b=0;
     }else{b=1;}


}

void CCPPWM_init(){                  // Configure and CCP2 at 2ms period with 50% duty cycle
        T2CON = 0x07;                    // Enable Timer2 at Fosc/4 with 1:16 prescaler (8 uS percount 2000uS to count 250 counts)
        CCP2CON = 0x0C;                  // Enable PWM for CCP2
        PR2 = 250;                       // 250 counts = 8uS *250 = 2ms period
        CCPR2L = 125;                    // Buffer where we are specifying the pulse width (duty cycle)
}


void Speed(int p){
       CCPR2L = p;                  // PWM from RC1
}

void mymsDelay(int x);

void ATD_init_A0();
unsigned int ATD_read_A0();

void check_front_right();
void check_front_left();
void check_front();


void main() {

     initialize();
     ATD_init_A0();
     CCPPWM_init();
     b=0;
     a=0;
     while(1){
        PORTD=PORTD & 0B11011111;
        check_start();

             while(b){
             PORTD=PORTD | 0B00100000;
             check_front_right();
             check_start();
             check_front_left();
             check_start();
             check_front();
             check_start();
             adjust_position();
             check_start();
             ENDD();
             check_start();
 }                    }
}



void initialize(){
TRISA=0X01;
TRISB=0X00;
TRISC=0B10000000;
TRISD=0B00001111;

PORTA=0X00;
PORTB=0X00;
PORTC=0X00;
PORTD=0X00;


   OPTION_REG= 0x87;//Use internal clock Fosc/4 with a prescaler of 256
   TMR0=248;// will count 8 times before the overflow (8* 128uS = 1ms)
   INTCON = 0b11100000; //GIE and , T0IE, peripheral interrupt

   T1CON=0x01;
   TMR1H=0;
   TMR1L=0;

   CCP1CON=0x08;
   PIE1=PIE1|0x04;// Enable CCP1 interrupts
   CCPR1H=2000>>8;
   CCPR1L=2000;

   H_L = 1;
}





void interrupt(){
    if(INTCON & 0x04){// TMR0 Overflow interrupt, will get here every 1ms
       TMR0=248;
       tick++;
       tick1++;
       check_start();
       INTCON = INTCON & 0xFB;//Clear T0IF
       }
       
if(PIR1&0x04){//CCP1 interrupt
   if(a==1){                                           // CCP1 interrupt
             if(H_L){                                // high
                       CCPR1H = angle >> 8;
                       CCPR1L = angle;
                       H_L = 0;                      // next time low
                       CCP1CON = 0x09;              // compare mode, clear output on match
                       TMR1H = 0;
                       TMR1L = 0;
             }
             else{                                          //low
                       CCPR1H = (40000 - angle) >> 8;       // 40000 counts correspond to 20ms
                       CCPR1L = (40000 - angle);
                       CCP1CON = 0x08;             // compare mode, set output on match
                       H_L = 1;                     //next time High
                       TMR1H = 0;
                       TMR1L = 0;
             }

             PIR1 = PIR1&0xFB; }else{

              PIR1 = PIR1&0xFB;
             }
       }

}




void ATD_init_A0(){
ADCON0 = 0x41; // ATD ON, Dont go, channel 0, fosc/16
ADCON1 = 0xCE; // All channels are digital except A0 , 500 khz , right justified
}


unsigned int ATD_read_A0(){
ADCON0 = ADCON0 | 0x04; // GO
while(ADCON0 & 0x04);
return ((ADRESH<<8) | ADRESL);
}


void mymsDelay(int const x){
       tick=0;
       while(tick<x);
}

void check_front_right(){
 // read port D0 : right sensor
if(!(PORTD & 0b00000001)){
    tick1 = 0;
    // read port D2 : front sensor
    while((PORTD & 0b00000100)){
    // if it turns more than the turning_th stop (turning_th is time)
    if (tick1 >= 4000) break;
    // move right
    move_right();

    }
    mymsDelay(100);
// stop moving
stop_moving();
}
 }


void check_front_left(){
// read port D1 : left sensor
if (!(PORTD & 0b00000010)){
     tick1 = 0;
    // read port D2 : front sensor
    while((PORTD & 0b00000100)){
      // move left
      if (tick1 >= 4000) break;
      move_left();
      }
// stop moving
stop_moving();
}
}

void check_front(){
while(!(PORTD & 0b00000100)){
sensor_voltage = ATD_read_A0();

 while(sensor_voltage <= 70 || sensor_voltage >= 100){
   sensor_voltage = ATD_read_A0();
 move_forward();
 distance=dist();
 if(distance<20){
 move_right();
 mymsDelay(2000);
 move_forward();
 mymsDelay(2000);
 move_left();
 mymsDelay(2000);
 move_forward();
 }else{move_forward();}
  if(PORTD & 0b00000100){break;}
 }
 if(sensor_voltage <= 100) {break;}
 if(PORTD & 0b00000100){break;} }

  mymsDelay(100);
  stop_moving();
}



void stop_moving(){
     PORTB = PORTB & 0b11110000;
}

void move_left(){

      Speed(90);
      PORTB =(PORTB & 0b11110000)| 0b00001001;
}

void move_right(){
     Speed(90);
     PORTB = (PORTB & 0b11110000) | 0b00000110;
}

void move_forward(){
     Speed(140);
     PORTB = (PORTB & 0b11110000)| 0b00000101;

}

void move_backwards(){
     Speed(90);
     PORTB = (PORTB & 0b11110000)| 0b00001010;
}



void adjust_position(){
while(!(PORTD & 0b00000100)){
     sensor_voltage = ATD_read_A0();
     while(sensor_voltage < 70){
     move_backwards();
     sensor_voltage = ATD_read_A0();
}

if( sensor_voltage>=70) {break;}
if(PORTD & 0b00000100){break;}
}
mymsDelay(100);
stop_moving();
}




void ENDD(){

 sensor_voltage = ATD_read_A0();
 while (!(PORTD & 0b00000100))
 {
       if(sensor_voltage <70 || sensor_voltage > 100) break;
       stop_moving();
       a=1;
       angle = 1000;
       PORTD=PORTD | 0B11000000;
       mymsDelay(2000);
       angle = 3500;
       PORTD=PORTD & 0B00111111;
       mymsDelay(2000);
       angle = 1000;
       PORTD=PORTD | 0B11000000;
       mymsDelay(2000);
       angle = 3500;
       PORTD=PORTD & 0B00111111;
       mymsDelay(2000);
       angle = 1000;
       PORTD=PORTD | 0B11000000;
 }
 PORTD=PORTD & 0B00111111;
angle =  2250;
a=0;
}

int dist(){
    int d = 0;
    T1CON = 0x10; // Use internal clock, no prescaler
    mymsDelay(200);

    T1CON = 0x10;
    TMR1H = 0;                  // Reset Timer1
    TMR1L = 0;

    PORTC = PORTC | 0b01000000; // Trigger HIGH
    delay_us(10);               // 10 µs delay
    PORTC = PORTC & 0b10111111; // Trigger LOW

    while (!(PORTC & 0b10000000));


    T1CON = T1CON | 0b00000001; // Start Timer


    while (PORTC & 0b10000000);

    T1CON = T1CON & 0b11111110; // Stop Timer

    d = (TMR1L | (TMR1H << 8)); // Read Timer1 value
    d = d / 58.82;           // Convert time to distance (cm)

    mymsDelay(10);
    T1CON = 0x01;
    return d;
}
