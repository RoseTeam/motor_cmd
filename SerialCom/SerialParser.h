#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include "SerialCom.h"
#include "MotorCtrl.h"
#include "Metrics.h"


#define KP_POLAR_ANGLE 1
#define KD_POLAR_ANGLE 0.0
#define KI_POLAR_ANGLE 0.0
#define KI_POLAR_ANGLE_SAT 1.0

#define KP_POLAR_LINEAR 0.25
#define KD_POLAR_LINEAR 0.0
#define KI_POLAR_LINEAR 0.000
#define KI_POLAR_LINEAR_SAT 1.0

#define COM_TIMEOUT 0.4  //seconde

class SerialParser: public SerialCom
{
    public:
    SerialParser(PinName tx, PinName rx , MotorCtrl& _asser);

    bool interpretData(); //Function to be called when we received the full packet    
    void printOdo(); // print data like odometri or metrics
    void printRobotStatus();
    void resetComData();
    bool checkComTimeout();
    bool verifyCs(char* buff);
    void initCst();
    
    void printDebug(char* message);
    void printTime(double msg);
    void printServoing();

    int Xorder;  // Commands data from the PC/ROS node
    int Yorder;
    int Aorder;
    float Lspeed;
    float Rspeed;
    float Ttwist;
    float Vtwist;
    int SStatus;
    bool UPower;
    float distance;
    float angle;
    float finalDistanceSpeed;
    float finalAngleSpeed;
    int uuid;
    int mode;
    
    
    float StartX;
    float StartY;
    float StartTheta;

    float KpPoLin;  //Polar, linear proportional coefficient
    float KiPoLin;
    float KdPoLin;
    float KiPoLinSat; //Saturation value for integral term

    float KpPoAng;  //Polar, angular proportional coefficient
    float KiPoAng;
    float KdPoAng;
    float KiPoAngSat; //Saturation value for integral term
    
    MotorCtrl& asser;
    
    
    bool odoEnabled;
    
    Timer com_timeout;
    
};

#endif // SERIALPARSER_H
