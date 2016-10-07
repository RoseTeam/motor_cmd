#include "SerialParser.h"

bool SerialParser::parseMsg(char const* msg)
{
	float tmp;
	if (1 == sscanf(msg + 1, "%f", &tmp))
	{
		data_[static_cast<uchar>(msg[0])] = tmp;
		return true;
	}
	return false;
}

void SerialParser::setMsg(MsgType msgType)
{
	sprintf(outgoingMsg_, "%c%f\n", msgType, getData(msgType));
}
