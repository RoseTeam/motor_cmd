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
DigitalOut led(LED2);

int main()
{
	ModSerialParser serialParser;
	
	for(int i = 0; ; i++)
	{
		led = i % 2;
	
		wait_ms(500);

		//serialParser.comPC.printf("coucou %d\n", i);

		serialParser.odom.a = i;
		serialParser.sendMsg(MsgType::Odom);

		serialParser.Aorder = -i;

		serialParser.sendMsg(MsgType::Aorder);
	}
}
