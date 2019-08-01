///---Define's---///
#define MAX_SPEED 150  //Velocidad Maxima (maximo valor posible 255)
#define MED_SPEED  80  //Velocidad en rectas  (maximo valor posible 255)
#define OUT_LINE   16   //Valor fijo dado cuando se sale de linea
#define KP         12   //Constante P
  

enum {HOME,STARTING,RUN} state = HOME;
enum {CENTER,RIGHT,LEFT} out_state = CENTER;

int pin2 = 0,pin3=0,pin4=0,pin5=0,pin6=0,pin7=0,pin8=0,pin9=0,pin10=0,pin11=0,pin12=0,pin14=0,pin15=0,pin16=0,pin17=0,pin18=0,pin19=0;
   //Local variables//
   signed int error_actual = 0;
    signed int error_anterior = 0;
   
   signed int speed_1  =0; //Motor 2 - left side
    signed int speed_2  =0; //Motor 1 - right side
   
   signed int proporcional = 0;
    
void setup() {
  // Configuramos pines
pinMode(2, INPUT); //sensor
pinMode(3, OUTPUT); //PWM1
pinMode(4, INPUT);  //Sensor
pinMode(5, INPUT);  //sensor
pinMode(6, INPUT);  //sensor
pinMode(7, INPUT);  //sensor
pinMode(8, INPUT);  //sensor

pinMode(9, OUTPUT); //Direccion PWM1
pinMode(10, OUTPUT);  //PWM2
pinMode(11, OUTPUT);  //Direccion PWM2

pinMode(12, INPUT); //sensor
pinMode(13, OUTPUT); //led

pinMode(14, INPUT); //sensor
pinMode(15, INPUT); //sensor
pinMode(16, INPUT); //sensor
pinMode(17, INPUT); //sensor
pinMode(18, INPUT); //S2
pinMode(19, INPUT); //S1

//Serial.begin(9600);

analogWrite(3,0); //PWM1 487Hz  DC. de 0 a 255
analogWrite(10,0); //487Hz PWM2
digitalWrite(9, HIGH);//direccion PWM1
digitalWrite(11, HIGH); //Direccion PWM2


}

///---Functions---///
///Function to read error///
signed int Read_error (void)
{
    int b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10;
    signed int error = 0;
    //Lectura de sensores de manera digital
   b0=(digitalRead(8));
   b1=(digitalRead(7));
   b3=(digitalRead(6));
   b2=(digitalRead(12));
   b4=(digitalRead(14));
   b5=(digitalRead(5));
   b6=(digitalRead(15));
   b7=(digitalRead(4));
   b8=(digitalRead(16));
   b9=(digitalRead(2));
   b10=(digitalRead(17));

  
   if (b0||b1||b2||b3||b4||b5||b6||b7||b8||b9||b10) {  //Out line - Black line (si alguno lee linea entra)
   
      /*Negative left sensor*/
        error = (b1)         ? (0-2) : error; 
        error = (b3)         ? (0-4) : error;
        error = (b5)         ? (0-6) : error;
        error = (b7)         ? (0-8) : error;
        error = (b9)         ? (0-10) : error;
        error = (b1 && b3)   ? (0-3) : error; 
        error = (b3 && b5)   ? (0-5) : error;
        error = (b5 && b7)   ? (0-7) : error;
        error = (b7 && b9)   ? (0-9) : error;
        
        
        /*Positive right sensor*/
        error = (b2)         ? 2 : error;
        error = (b4)         ? 4 : error;
        error = (b6)         ? 6 : error;
        error = (b8)         ? 8 : error;
        error = (b10)        ? 10 : error;
        error = (b2 && b4)   ? 3 : error;
        error = (b4 && b6)   ? 5 : error;
        error = (b6 && b8)   ? 7 : error;
        error = (b8 && b10)  ? 9 : error;
        
        
        /*Neutral middle sensor*/
        error = (b0)         ? 0 : error;
        error = (b0 && b1)   ? (0-1) : error;
        error = (b0 && b2)   ? 1 : error;
        
        out_state = ((error <= 4)&&(error >=(0-4)))  ? CENTER : out_state; 
        out_state = ((error >= 5)&&(error <=10))        ? LEFT : out_state;
        out_state = ((error <=(0-5))&&(error >=(0-10))) ? RIGHT: out_state;

        return error;
    }
      else {
    return OUT_LINE;
    }
}

///Function to set speed of right motor///
void Motor_R (signed int speed)
{
   if(!speed) {
   
   analogWrite(3,0); // M1 Duty 0% 
   
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      
      if (speed >=1){
      analogWrite(3,speed);  
      digitalWrite(9, HIGH);
            
      
      }
      else {
      speed *= (0-1);
      ////////////////////////
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      ///////////////////////////////////
      analogWrite(3,speed); 
      digitalWrite(9, LOW); //Cambio de giro
      
      }
   } 
   return;
}

///Function to set speed of left motor///
void Motor_L (signed int speed)
{  
   if(!speed) {
   analogWrite(10,0);
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      
      if (speed >=1){
      analogWrite(10,speed);  
      digitalWrite(11, LOW); //high adelante M1
      }
      else {
      speed *= (0-1);
      ////////////////////////
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      ///////////////////////////////////
      analogWrite(10,speed);
      digitalWrite(11, HIGH); //high adelante M1
      }
   }
   return;
}

void loop() {
  
switch (state)
      {
         case HOME:
            Motor_R(0);
            Motor_L(0);
            
            if(digitalRead(18) == 1){ //se queda esperando a que se presione el boton
               state = STARTING;
            }
            delay(100);
            
            break;
         
         case STARTING:
            digitalWrite(13, HIGH);
            delay(500);
            digitalWrite(13, LOW);
            delay(500);
            digitalWrite(13, HIGH);
            delay(300);
            state = RUN;
            break;
            
         case RUN:
            //Read real error -10 to 10
            error_actual = Read_error();
            
            //Out line
            if(error_actual == OUT_LINE) {
               //Out line state machine//
               switch (out_state)
                    {
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
            //On line
            else {
                    
                    proporcional = (KP * error_actual);
              
                  
                    speed_1  = MED_SPEED + proporcional;
                    speed_2  = MED_SPEED - proporcional;
            }
            
            Motor_R(speed_2);
            Motor_L(speed_1);
            
            break;
      }
   

}
