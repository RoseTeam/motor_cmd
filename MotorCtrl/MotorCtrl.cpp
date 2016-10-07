#include "MotorCtrl.h"
#include "SerialParser.h"
#include "mbed.h"

#define DEUXPI 6.28318530718
#define PI 3.14159265359

MotorCtrl::MotorCtrl() :
wheelL(Encoder1_A, Encoder1_B, NC, ENCODER_RES),wheelR(Encoder2_A, Encoder2_B, NC, ENCODER_RES), 
PidAngle(&angle, &pidAngleOutput, &angleCommand.target,KP_POLAR_ANGLE, KI_POLAR_ANGLE, KD_POLAR_ANGLE,-1),
PidDistance(&distance, &pidDistanceOutput, &distanceCommand.target,KP_POLAR_LINEAR, KI_POLAR_LINEAR, KD_POLAR_LINEAR,-1),
angle_buf(4), distance_buf(4)

{
    ODO_X = START_COORD_X ;
    ODO_Y = START_COORD_Y;
    ODO_Theta = START_COORD_Theta;

    resetOdometryRequested = false; //used to trigger an odomtery reset on the next motor control loop

    tickToDistance = (3.14159*R_WHEEL/ENCODER_RES)*54.4/100;
    
    tickToAngle =  3.14159*R_WHEEL/(WHEEL_B * ENCODER_RES)* 6.28328530718/5.569498538970947;
    previous_ODO_Theta = 0.0;
    
    
    distanceCommand.SetStepPerMeter(1.0/tickToDistance);
    angleCommand.SetStepPerMeter(1.0/tickToAngle);
    
    isEnabled = true;
    commandUUID = 0;
 // variable de position initialisées
          
    mode_deplacement = 1;// sert à déffinir le mode de déplacement : polaire, linèaire...(ici 1 seul mode)
    
    loop_frequency.start();
    
}



void MotorCtrl::ComputeOdometry()
{
    if(resetOdometryRequested)
    {       
        __disable_irq();  //disable every interrupts
    
        wheelL.reset();
        wheelR.reset();
        
        wheelLTick = wheelL.getPulses();
        wheelRTick = wheelR.getPulses();
    
        distance_buf.put((wheelLTick + wheelRTick ));
        angle_buf.put((wheelRTick - wheelLTick));
             
        ODO_X = _parser->StartX; // START_COORD_X ;
        ODO_Y = _parser->StartY; 
        ODO_Theta_Init = _parser->StartTheta;
    
        ODO_SPEED_X = 0.0;
        ODO_SPEED_Y = 0.0;
        ODO_SPEED_Theta = 0.0;
        ODO_SPEED_Lin = 0.0; //variables de vitesses
        
        ODO_DELTA_X = 0.0;
        ODO_DELTA_Y = 0.0;
        
         deltaDistance = 0;
         distance =0 ;
         previousDistance = 0;
         angle = 0;
      
        __enable_irq();  //enable every interrupts
                
        resetOdometryRequested = false;
    }
    
    else if (DataAvailable())
        {
        deltaDistance = distance - previousDistance;
        previousDistance = distance;
        double trueDeltaDistance = deltaDistance * tickToDistance;
        
        //ODO_ds = deltaDistance/ENCODER_RES;// angle rotation en radians!!!
        
        ODO_Theta = tickToAngle * angle + ODO_Theta_Init; //ODO_Theta_Offset; //  mesure la rotation en radian
        
        ODO_DELTA_X = trueDeltaDistance  * cos(ODO_Theta);
        ODO_DELTA_Y = trueDeltaDistance* sin(ODO_Theta);  
    
        ODO_X = ODO_X + ODO_DELTA_X;
        ODO_Y = ODO_Y + ODO_DELTA_Y;
        
        float odom_freq = loop_frequency.read();
        
        ODO_SPEED_Lin = trueDeltaDistance/odom_freq;///odom_freq;
        ODO_SPEED_X = ODO_DELTA_X/odom_freq;
        ODO_SPEED_Y = ODO_DELTA_X/odom_freq;
        ODO_SPEED_Theta = (ODO_Theta-previous_ODO_Theta)/odom_freq;
        previous_ODO_Theta = ODO_Theta;
        loop_frequency.reset();   //computes the time since the last control loop!
        }
}

void MotorCtrl::setX(float x){
    ODO_X = x;
}

void MotorCtrl::setY(float y){
    ODO_Y = y;
}

void MotorCtrl::setAngle(float Theta){
 
}


void MotorCtrl::setRequestOdometry(bool value){
    resetOdometryRequested = value;
}




bool MotorCtrl::getRequestOdometry(){
    return resetOdometryRequested;
}



void MotorCtrl::setTarget(float distance, float angle, float finalDistanceSpeed, float finalAngleSpeed, int uuid){
    commandUUID = uuid;
    distanceCommand.setTarget(distance,finalDistanceSpeed);
    angleCommand.setTarget(angle,finalAngleSpeed); 
}

void MotorCtrl::setTargetAngle(float angleAbs, int uuid){
    commandUUID = uuid;
    double angle =  angleAbs - ODO_Theta;
    
    if (angle  > 0.0){
        angle = fmod(angle + PI, 2.0 * PI) - PI;
    } else {
        angle = fmod(angle - PI, 2.0 * PI) + PI;
    }
    angleCommand.setTarget(angle, 0);
}

void MotorCtrl::setTargetXY(float x, float y, float finalDistanceSpeed, int mode, int uuid){ //mode rotation = 1, distance = 2 , both = 3
    commandUUID = uuid;
    x -= ODO_X;
    y -= ODO_Y;
    
    float dist = 0.0;
    float angle = 0.0;
    
    if (x != 0.0 || y != 0.0){
        dist = sqrt(x * x + y * y);
        angle = atan2(y,x) - ODO_Theta;

        if (angle  > 0.0){
            angle = fmod(angle + PI, 2.0 * PI) - PI;
        } else {
            angle = fmod(angle - PI, 2.0 * PI) + PI;
        }
    }
    if (mode & 0x2){
        distanceCommand.setTarget(dist, finalDistanceSpeed);
    }
    if (mode & 0x1){
       angleCommand.setTarget(angle,0);
    }  
}

void MotorCtrl::setTickToAngle(float pTickToAngle){
    tickToAngle= pTickToAngle;
    angleCommand.SetStepPerMeter(1.0/pTickToAngle);
} 

void MotorCtrl::setTickToDistance(float pTickToDistance){
    tickToDistance = pTickToDistance;
    distanceCommand.SetStepPerMeter(1.0/pTickToDistance);
}

void MotorCtrl::enable(int is_enabled){
    isEnabled = is_enabled;
    
    //distanceCommand.Reset(distance); // reset speed trajectory generator
    //angleCommand.Reset(angle);
    
}

void MotorCtrl::resetServoingParam(){
    resetOdometryRequested = true;  
}
  
void MotorCtrl::SystemCtrl(){     
 
    float setpointAspeed =  _parser->Ttwist;
    float setpointLspeed =  _parser->Vtwist;
    
    float feedbackAspeed = getODO_SPEED_Theta();    
    float feedbackLspeed = getODO_SPEED_Lin();
    
    
    orien = Compute_PID_Angle(feedbackAspeed, setpointAspeed);
    dist = Compute_PID_Linear(feedbackLspeed, setpointLspeed);
    
    float motorR = dist - orien;
    float motorL = dist + orien;
   
    
    if(isEnabled){
        Motors.Motor1(motorL);
        Motors.Motor2(motorR);
    }
    else {
        Motors.Motor1(0);
        Motors.Motor2(0);
    }
}


#define MAX_I_TERM_ANGLE 0.5


float MotorCtrl::Compute_PID_Angle(float feedbackAspeed, float setpointAspeed)
{
     float error = setpointAspeed - feedbackAspeed;
     static float old_error = 0.0;
     float error_dif = (error - old_error)*loop_frequency.read();
     old_error = error;
     
     float static error_sum = error_sum + error*loop_frequency.read();
     
     if (error_sum > MAX_I_TERM_ANGLE)
        {   error_sum = MAX_I_TERM_ANGLE;   }
     else if (error_sum < -MAX_I_TERM_ANGLE)
        {  error_sum = -MAX_I_TERM_ANGLE;   }   
        
                     
     float P = error*_parser->KpPoAng;
     float D = error_dif * _parser->KdPoAng;
     float I = error_sum* _parser->KiPoAng;
     
     float res = P + I + D;
     
     return res;      
}

#define MAX_I_TERM_LINEAR 0.5

float MotorCtrl::Compute_PID_Linear(float feedbackLspeed, float setpointLspeed)
{
     float error = setpointLspeed - feedbackLspeed;
     static float old_error = 0.0;
     float error_dif = (error - old_error)*loop_frequency.read();;
     old_error = error;
     
     float static error_sum = error_sum + error*loop_frequency.read();
     
     if (error_sum > MAX_I_TERM_LINEAR)
        {   error_sum = MAX_I_TERM_LINEAR;   }
     else if (error_sum < -MAX_I_TERM_LINEAR)
        {  error_sum = -MAX_I_TERM_LINEAR;   }     
     
     float P = error*_parser->KpPoLin;
     float D = error_dif * _parser->KdPoLin;
     float I = error_sum* _parser->KiPoLin;
     
     float res = P + I + D;
        
     return res;      
}


void MotorCtrl::Interrupt_Handler()
{
    wheelLTick = wheelL.getPulses();
    wheelRTick = wheelR.getPulses();
    
    distance_buf.put((wheelLTick + wheelRTick ));
    angle_buf.put((wheelRTick - wheelLTick));
}

bool MotorCtrl::DataAvailable(){
    if (distance_buf.available()){
       distance = distance_buf;
       angle = angle_buf;
       return true;
    }
    return false;
}

void MotorCtrl::Compute()
{
 // compute target
    //distanceCommand.Compute();
    //angleCommand.Compute();
    //distanceCommand.blockageDetector(distance);
    //angleCommand.blockageDetector(angle);
                                          
    SystemCtrl();
}

double MotorCtrl::getODO_X(){
    return ODO_X;
}

double MotorCtrl::getODO_Y(){
    return ODO_Y;
}

double MotorCtrl::getODO_Theta(){
    return ODO_Theta;
}

double MotorCtrl::getODO_SPEED_Lin(){
    return ODO_SPEED_Lin;
}

double MotorCtrl::getODO_SPEED_X(){
    //return ODO_DELTA_X/(loop_frequency.read_us()/1000000);
    return ODO_SPEED_X;
}

double MotorCtrl::getODO_SPEED_Y(){
    return ODO_SPEED_Y;
}

double MotorCtrl::getODO_SPEED_Theta()
{
    return ODO_SPEED_Theta;
}

long MotorCtrl::getWheelL()
{
    return wheelLTick;
}

long MotorCtrl::getWheelR()
{
    return wheelRTick;
}

void MotorCtrl::Debug(){
    
      //ComPC.sendFeedback(ODO_X,ODO_X,pidA,pidT);
        
}
