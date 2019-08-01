#include "line.h"
FollowerLine Follower;
//kp 13.5
void setup() {
  //SetSpeeds(median speed,maximum speed);
  Follower.SetSpeeds(80,150);
  //SetConstantsController(KP,KI,KD)
  Follower.SetConstantsController(13.5,0,15);
  //SetColorLine(0);//Line black 0 or Line white 1
  Follower.SetColorLine(1);
  // put your setup code here, to run once:
  Follower.InitFollower();
}
void loop() {
  Follower.StateMachine();
}
ISR(TIMER0_COMPA_vect){ 
  if(Follower.control_switch){
    Follower.ReadSensors();
    Follower.ControllerPID();
  }
}
