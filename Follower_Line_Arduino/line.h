#ifndef LINE_H
#define LINE_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>    
#include <stdbool.h>  

class FollowerLine
{
  public:
        FollowerLine();
        void InitFollower();
        void ReadSensors();
        void StateMachine();
        void Motor_R (signed int speed);
        void Motor_L (signed int speed);
        void ControllerPID();
        void SetColorLine(bool ColorLine);
        void SetConstantsController(float KP,float KI,float KD);
        void SetSpeeds(int med_speed,int max_speed);
        bool control_switch;
        
  private:  
        signed int error;
        bool line;
        bool b[11];
        bool color_line;
        enum {CENTER,RIGHT,LEFT}out_state;   
        enum {STOP,STARTING,RUN} state;
        signed int u;
        float Kp;
        float Ki;
        float Kd;
        signed int speed_1;  
        signed int speed_2;
        float x[3];
        int flag;   
        int MAX_SPEED; 
        int MED_SPEED; 
  };
#endif
