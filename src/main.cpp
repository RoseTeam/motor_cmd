#include "mbed.h"
#include <MODSERIAL.h>
#include "SerialParser.h"

struct ModSerialParser : public SerialParser<MODSERIAL>
{
	MODSERIAL comPC{ SERIAL_TX, SERIAL_RX };

	ModSerialParser() : SerialParser<MODSERIAL>(comPC) {
		comPC.baud(115200);

		comPC.autoDetectChar(delimiter_end_);
		comPC.attach(this, &ModSerialParser::modserialCallback, MODSERIAL::RxAutoDetect);
	}

	// This method is called when a character goes into the TX buffer.
	void modserialCallback(MODSERIAL_IRQ_INFO *) {
		
		auto buf_size = rx_buffer_.size();
		auto buf_count = comPC.rxBufferGetCount();
		rx_buffer_.resize(buf_size + buf_count);
		comPC.move(&rx_buffer_[buf_size], buf_count);
		parseMsg();
	}

};


int main()
{
	ModSerialParser serialParser;
	
	for(int i = 0; ; i++)
	{
		wait_ms(300);

		serialParser.odom.a = i;
		serialParser.sendMsg(MsgType::Odom);

		serialParser.Aorder = -i;

		serialParser.sendMsg(MsgType::Aorder);
	}
}
