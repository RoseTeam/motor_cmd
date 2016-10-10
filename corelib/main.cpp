#include "SerialParser.h"
#include <vector>

struct DummySerialHelper
{
	std::vector<char> rxMsg_;

	void putc(int c)
	{
		printf("char:{%c}",static_cast<char>(c));
		rxMsg_.push_back(c);		
	}

};

int main()
{
	DummySerialHelper serial_helper;
	SerialParser<DummySerialHelper> ser(serial_helper);

	ser.Aorder = 0;
	printf("%f\n", ser.Aorder);

	ser.parseMsg();
	ser.sendMsg(MsgType::Aorder);
	
	ser.Aorder = 2.2f;
	printf("%f\n", ser.Aorder);

	ser.setMsg(&ser.serial_helper_.rxMsg_[0], ser.serial_helper_.rxMsg_.size());
	ser.parseMsg();

	
	printf("%f\n", ser.Aorder);	
}