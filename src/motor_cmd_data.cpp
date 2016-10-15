#include "motor_cmd_data.h"

#define CASE_SET_DATA_REF(msgType)\
	case MsgType::msgType:\
	obj = reinterpret_cast<char*>(&this->msgType);\
	size = sizeof(this->msgType);\
	break

void MotorCmdData::getDataRef(MsgType msgType, char* & obj, int& size)
{
	switch (msgType)
	{
		CASE_SET_DATA_REF(Twist);
		CASE_SET_DATA_REF(Odom);
		CASE_SET_DATA_REF(pidPropulsion);
		CASE_SET_DATA_REF(pidDirection);
		CASE_SET_DATA_REF(odomPeriod);
		CASE_SET_DATA_REF(Status);
		CASE_SET_DATA_REF(msgReceived);
		CASE_SET_DATA_REF(msgCorrupted);
		CASE_SET_DATA_REF(bytesReceived);

	default:
		obj = nullptr;
		size = 0;
	}
}