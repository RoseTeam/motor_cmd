#pragma once

typedef unsigned char uchar;

struct Vect2d
{
	float x{ 0 };
	float y{ 0 };
};

struct PIDCoeffs
{
	float kP{ 0 };
	float kI{ 0 };
	float kD{ 0 };
};

struct TwistMsg
{
	Vect2d pos;
	float theta{ 0 };
};

struct OdomMsg
{
	Vect2d pos;
};

/*
MsgType: identifier codes (1 byte) for data that can be serialized.

when adding a new type, three places needs to be touched:
1- add a value in MsgType enum
2- add a member variable with the same name as the enum in MotorCmdData struct
3- add a 'case' statement in MotorCmdData::getRefData using the macro in motor_cmd_data.cpp

(optionally if using python bindings :
- expose the enum value + data member in the pybind11 bindings
- if using a custom type (like Vect2d), expose that type also
)
*/
enum class MsgType : uchar
{
	Twist=0,
	Odom,
	odomPeriod,
	pidPropulsion,
	pidDirection,
	Status,
	msgReceived,
	msgCorrupted,
	bytesReceived,
	// special value used in deserialization, meaning no message/data
	// MUST BE THE LAST VALUE in enum
	NONE 
};
/*
enum class StatusCode : uchar
{
	STOP = 0,
	RUN = 1,
	ECHO = 2
};
*/
struct MotorCmdData
{
	TwistMsg Twist;
	OdomMsg Odom;

	int odomPeriod{ 10 }; // in ms

	PIDCoeffs pidPropulsion;
	PIDCoeffs pidDirection;

	uchar Status{ 0 };

	int msgReceived{ 0 };
	int msgCorrupted{ 0 };
	int bytesReceived{ 0 };

	void getDataRef(MsgType msgType, char* & obj, int& size);
};