#include "line.h"
FollowerLine::FollowerLine(){
               error=0;
               line=0;
               for(int i=0;i<11;i++){
                   b[i]=0;
               }
               out_state=CENTER;   
               state=STOP;
               u=0;
               Kp=13.5;
               Ki=0;
               Kd=15;
               speed_1=0;  
               speed_2=0;
               for(int i=0;i<3;i++){
                   x[i]=0;
               }
               flag=0;  
               MAX_SPEED=150; 
               MED_SPEED=80;
               color_line=1;
               control_switch=0;
  }

void FollowerLine::InitFollower(){
    DDRD &= ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4)| (1 << 2));
    DDRC &= ~((1 <<3) | (1 <<2) | (1 <<1) | (1 <<0));
    DDRB &= ~((1 <<4) | (1 <<0));
    DDRD |= (1 << 3);//PWM1
    DDRB |= (1 << 1); //Direccion PWM1
    DDRB |= (1 << 2);//PWM2
    DDRB |= (1 << 3);  //Direccion PWM2 
    cli();
    // ***********   PWM 1 Timer2 Correct phase   490 hz**************
    TCCR2A |= 0b00100001;//PWM by pin OC1B no inverted 8 bits correct phase
    TCCR2B |= 0b00000100;//Resolution of 8 bits and prescaler of 64
    OCR2B=0;//PWM dutly cycle
    PORTB |= (1 << PB1);//direccion PWM1
    // ***********   PWM 2 Timer1 Correct phase   490 hz**************
    TCCR1A |= 0b00100001;//PWM by pin OC1B no inverted 8 bits correct phase
    TCCR1B |= 0b00000011;//Resolution of 8 bits and prescaler of 64
    OCR1B=0;//PWM dutly cycle
    PORTB |= (1 << PB3);//Direccion PWM2
        //************** Timer Interrupt 2Khz***********************************
    TCCR0A = 0;// set entire TCCR0A register to 0
    TCCR0B = 0;// same for TCCR0B
    TCNT0  = 0;//initialize counter value to 0
    // set compare match register for 1khz increments
    //OCR0A = 249;// = [(16*10^6) / (1000*prescaler)] - 1 (must be <256)
    // set compare match register for 0.5khz increments
    OCR0A = 124;// = [(16*10^6) / (1000*prescaler)] - 1 (must be <256)//245
    TCCR0A |= (1 << WGM01);// turn on CTC mode
    TCCR0B |= (1 << CS01) | (1 << CS00);   // Set CS01 and CS00 bits for 64 prescaler   
    //TCCR0B |= (1 << CS02);//Precaler 256  
    TIMSK0 |= (1 << OCIE0A);
    sei();
  }

void FollowerLine::ReadSensors(){
        if(color_line){
           b[0] = (PINB & (1 << PB0));
           b[1] = (PIND & (1 << PD7));
           b[2] = (PINB & (1 << PB4));
           b[3] = (PIND & (1 << PD6));
           b[4] = (PINC & (1 << PC0));
           b[5] = (PIND & (1 << PD5));
           b[6] = (PINC & (1 << PC1));
           b[7] = (PIND & (1 << PD4));
           b[8] = (PINC & (1 << PC2));
           b[9] = (PIND & (1 << PD2));
           b[10]=(PINC & (1 << PC3));
          }
          else{
           b[0] = !(PINB & (1 << PB0));
           b[1] = !(PIND & (1 << PD7));
           b[2] = !(PINB & (1 << PB4));
           b[3] = !(PIND & (1 << PD6));
           b[4] = !(PINC & (1 << PC0));
           b[5] = !(PIND & (1 << PD5));
           b[6] = !(PINC & (1 << PC1));
           b[7] = !(PIND & (1 << PD4));
           b[8] = !(PINC & (1 << PC2));
           b[9] = !(PIND & (1 << PD2));
           b[10] = !(PINC & (1 << PC3));
            }

    if (b[0]||b[1]||b[2]||b[3]||b[4]||b[5]||b[6]||b[7]||b[8]||b[9]||b[10]) { 
        error = (b[1])         ? (0-2) : error; 
        error = (b[3])         ? (0-4) : error;
        error = (b[5])         ? (0-6) : error;
        error = (b[7])         ? (0-8) : error;
        error = (b[9])         ? (0-10) : error;
        error = (b[1] && b[3])   ? (0-3) : error; 
        error = (b[3] && b[5])   ? (0-5) : error;
        error = (b[5] && b[7])   ? (0-7) : error;
        error = (b[7] && b[9])   ? (0-9) : error;
        /*Positive right sensor*/
        error = (b[2])         ? 2 : error;
        error = (b[4])         ? 4 : error;
        error = (b[6])         ? 6 : error;
        error = (b[8])         ? 8 : error;
        error = (b[10])        ? 10 : error;
        error = (b[2] && b[4])   ? 3 : error;
        error = (b[4] && b[6])   ? 5 : error;
        error = (b[6] && b[8])   ? 7 : error;
        error = (b[8] && b[10])  ? 9 : error;
        /*Neutral middle sensor*/
        error = (b[0])         ? 0 : error;
        error = (b[0] && b[1])   ? (0-1) : error;
        error = (b[0] && b[2])   ? 1 : error;
        out_state = ((error <= 4)&&(error >=(0-4)))  ? CENTER : out_state; 
        out_state = ((error >= 5)&&(error <=10))        ? LEFT : out_state;
        out_state = ((error <=(0-5))&&(error >=(0-10))) ? RIGHT: out_state;
        line=1;
    }
    else{
        line=0;
      }
}
void FollowerLine::Motor_R (signed int speed)
{
   if(!speed) {
     OCR2B=0; // M1 Duty 0% 
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      if (speed >=1){
      OCR2B=speed; 
      PORTB |= (1 << PB1);
      }
      else {
      speed *= (0-1);
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      OCR2B=speed; 
      PORTB&=~(1<<PB1);//Cambio de giro
      }
   } 
   return;
}

///Function to set speed of left motor///
void FollowerLine::Motor_L (signed int speed)
{  
   if(!speed) {
   OCR1B=0;
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      if (speed >=1){
      OCR1B=speed; 
      PORTB&=~(1<<PB3); 
      }
      else {
      speed *= (0-1);
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      OCR1B=speed;
      PORTB|=(1<<PB3); 
      }
   }
   return;
}
void FollowerLine::ControllerPID(){
    if(line) {
     //PID Control
    x[0]=error;
    x[2]=x[0]+x[2];
    u=Kp*x[0]+Ki*(x[2])+Kd*(x[0]-x[1]);
    speed_1=MED_SPEED+u;
    speed_2=MED_SPEED-u;
    Motor_R(speed_2);
    Motor_L(speed_1);
    x[1]=x[0];
  }
   else{
      switch (out_state){
      case CENTER:
           speed_1 = MED_SPEED;
           speed_2 = MED_SPEED;
           break;
      case LEFT:
           speed_1 = MAX_SPEED;
           speed_2 = (0-MAX_SPEED);
           break;
      case RIGHT:
           speed_1 = (0-MAX_SPEED);
           speed_2 = MAX_SPEED;
           break;
    }
    }
  }
void FollowerLine::StateMachine(){
   switch(state){
    case STOP:
            OCR1B=0;
            OCR2B=0;
            if(flag==0&&(PINC & (1 << PC4))){
                state=STARTING;
                flag++;
            } 
            break;
    case STARTING:
            OCR1B=MED_SPEED;
            OCR2B=MED_SPEED;
            state=RUN;
            break;
    case RUN:
            control_switch=1;
            break; 
     }
  }
void FollowerLine::SetConstantsController(float KP,float KI,float KD){
    Kp=KP;
    Ki=KI;
    Kd=KD;
  }
void FollowerLine::SetSpeeds(int med_speed,int max_speed){
    MED_SPEED=med_speed;
    MAX_SPEED=max_speed;
  }
void FollowerLine::SetColorLine(bool ColorLine){
    color_line=ColorLine;
  }
