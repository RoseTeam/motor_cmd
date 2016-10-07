#include "mbed.h"
#include "SerialParser.h"
#include "MotorCtrl.h"
#include "MotorDriver.h"

#define MAX(a,b) (((a)>(b))?(a):(b))

#define TICKER_ENCODER_PERIOD 4000  //us
#define TICKER_ROBOT_CTRL_PERIOD 10000  //us


DigitalOut myled(LED1);


MotorCtrl asser;  // handles the motor control algorithm (access to ComPc to know the orders)
//MotorDriver motors(MOTOR_1VIT,MOTOR_1DIR,MOTOR_2VIT,MOTOR_2DIR);
Metrics metrics;
//------------------------------------
// Hyperterminal configuration
// 230400 bauds, 8-bit data, no parity
//------------------------------------


SerialParser ComPC(PA_11, PA_12, asser); 
//SerialParser ComPC(SERIAL_TX, SERIAL_RX, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)
//SerialParser ComPC(PA_11, PA_12, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)

Ticker ticker_robot_ctrl;  //ticker to call motor control and odometry computation loop 
Ticker ticker_encoder_decoding;   //ticker to call the encoders buffers decoding function



Timer t_debug;
Timer t_ctrl;



int nbReceivedData = 0;
void tickerInterrupt()
{
    asser.Interrupt_Handler();
}


void robotControlInterrupt()
{
    
}

int main()
{    
    asser.setComm(&ComPC);
    
    ticker_encoder_decoding.attach_us(&tickerInterrupt, TICKER_ENCODER_PERIOD);
    //ticker_robot_ctrl.attach_us(&robotControlInterrupt, TICKER_ROBOT_CTRL_PERIOD);

    int msWaitTime = 18;
      
    t_ctrl.start(); 
    t_debug.start();
   
    //t_com.start();
    //t_debug.start();
    //t_perf.start();
    //TODO: Enable a WatchDog !!!

    while (1) {
       
        //ComPC.printDebug();
        //asser.Compute();
        
        //asser.ComputeOdometry();
        //asser.SystemCtrl();
   
     
     if ( t_ctrl.read_ms() > msWaitTime)
        {       
        t_ctrl.reset();
        t_debug.reset();
        
        ComPC.interpretData();
        ComPC.checkComTimeout();  
        
        
        
        asser.ComputeOdometry();   //computes the new position of the robot on the table
        asser.SystemCtrl();        // robot motion control function   
               
        ComPC.printOdo(); // print data like odometri or metrics
        //ComPC.printTime(t_debug.read()); 
        //ComPC.printServoing();
       
        }        
    }
}
