#include "SerialParser.h"


struct DummySerialHelper
{


};

int main()
{
	DummySerialHelper serial_helper;
	SerialParser<DummySerialHelper> ser(serial_helper);

	printf("%f\n", ser.getData(MsgType::Aorder));
	printf("tata%stutu\n", ser.getMsg());
	MsgData u;
	u.f = 4e8;
	ser.setPrepMsg(MsgType::Aorder, u.f);
	printf("tata%stutu\n", ser.getMsg());
	printf("%f\n", ser.getData(MsgType::Aorder));
	ser.setData(MsgType::Aorder, 1.11f);
	printf("%f\n", ser.getData(MsgType::Aorder));
	ser.parseMsg(ser.getMsg());
	printf("%f\n", ser.getData(MsgType::Aorder));

}