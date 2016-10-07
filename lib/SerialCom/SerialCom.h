
#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "mbed.h"
#include <MODSERIAL.h>

#define SC_STRING_LENGTH 64

#define BAUDRATE 115200


class SerialCom
{
	MODSERIAL modSerial_;

public:

    SerialCom(PinName tx, PinName rx);  //COnstructor

    void stateCheck();   // Function which changes internal variable when the satus of the robot has been updates

    char* getMessage();
    void releaseMessage();
    
    char message_buffer[SC_STRING_LENGTH];
    bool message_available;
    

private:
    int current_read_pos;


};

#endif
