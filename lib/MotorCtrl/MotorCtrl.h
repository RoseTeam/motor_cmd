#pragma once

#include "mbed.h"
#include "QEI.h"
#include "SerialCom.h"
#include "MotorDriver.h"
#include "pid.h"
#include "Asserv.h"
#include "Buffer.h"


#define Encoder1_A PC_12//D6//PA_3
#define Encoder1_B PC_10//D7//PA_2

#define Encoder2_A PB_1
#define Encoder2_B PB_2


#define TE 100                     // Temps d'échantiollonnage voulu  
#define R_WHEEL 0.025                // rayon des roues codeuses
#define WHEEL_B 0.188               // écartement des roues codeuses
#define ENCODER_RES 600.0          // Nombre de ticks par tour de roue


#define START_COORD_X 0.850          // coordonnées de démarrage du robot sur la table (zone de départ)
#define START_COORD_Y 0.230
#define START_COORD_Theta 0.0

//29.7 tick/mm


class SerialParser;


class MotorCtrl {
    
public:    

    MotorCtrl();  //Constructor
    
    Timer loop_frequency;
    
    void setComm(SerialParser * parser) 
    { _parser = parser; }
    
    void enable(int is_enabled);
    void resetServoingParam(); 
    bool DataAvailable();
    void Compute();
    void Interrupt_Handler();
   // void Control_interrupt();  //call temporal interrupt functions
    void ComputeOdometry();
    void SystemCtrl();
    
    
    void Debug();

    double getODO_X();
    double getODO_Y();
    double getODO_Theta();
    
	void setX(float x);
    void setY(float y);
    void setAngle(float Theta);
    void setTarget(float distance, float angle, float finalDistanceSpeed, float finalAngleSpeed, int uuid);
    void setTargetXY(float x, float y, float finalDistanceSpeed, int mode, int uuid); //mode rotation = 1, distance = 2 , both = 3
    void setTargetAngle(float angleAbs, int uuid);
    
    void setTickToAngle(float tickToAngle); 
    void setTickToDistance(float tickToDistance);
    
    void setRequestOdometry(bool value);
    bool getRequestOdometry();
    
      
    float Compute_PID_Angle(float feedbackAspeed, float setpointAspeed);
    float Compute_PID_Linear(float feedbackLspeed, float setpointLspeed);
    
    double getODO_SPEED_X();
    double getODO_SPEED_Y();
    double getODO_SPEED_Theta();
    double getODO_SPEED_Lin();
    
    long getWheelL();
    long getWheelR();

    float orien, dist;

    PID PidAngle;
    PID PidDistance;
    float pidAngleOutput;
    float pidDistanceOutput;
        
    Asserv distanceCommand;
    Asserv angleCommand;

    Buffer <uint32_t> angle_buf;
    Buffer <uint32_t> distance_buf;

    bool isEnabled;

    /*volatile*/ long encoder1Pos;   // ce sont les 2 entier ultra important sur lesquels repose les encodeur, l'odomètrie et l'asservicement
    /*volatile*/ long encoder2Pos;   // volatile ne sert à rien^^
    double encoder_position_old1;
    double encoder_position_old2; 
    
    float blockageDistanceTreshold;
    float blockageAngleTreshold;
    
    double ODO_X , ODO_Y , ODO_Theta, ODO_Theta_Init; // variable de position initialisées
    double previous_ODO_Theta;

    double ODO_SPEED_X, ODO_SPEED_Y, ODO_SPEED_Theta, ODO_SPEED_Lin; //variables de vitesses
    
    double tickToDistance;
    double tickToAngle;
    
    double ODO_DELTA_X , ODO_DELTA_Y;
    
    int commandUUID; // uid for sync between robot status and cmd sent
    int mode_deplacement;// sert à déffinir le mode de déplacement : polaire, linèaire...(ici 1 seul mode)
    int vitesse_roue_1, vitesse_roue_2, vitesse;
    
    QEI wheelL;
    QEI wheelR;
    
    int distance;
    int angle;
    
    int previousDistance;
    int previousAngle;
    int deltaAngle;
    int deltaDistance;
    
    int wheelLTick;
    int wheelRTick;
    
    bool resetOdometryRequested;
    
    MotorDriver Motors;
    
    SerialParser * _parser;
};
