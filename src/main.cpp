#include "mbed.h"
#include <MODSERIAL.h>
#include "SerialParser.h"

struct ModSerialParser : public SerialParser<MODSERIAL>
{
	MODSERIAL comPC{ SERIAL_TX, SERIAL_RX };

	ModSerialParser() : SerialParser<MODSERIAL>(comPC) {
		comPC.baud(115200);

		comPC.autoDetectChar(delimiter_);
		comPC.attach(this, &ModSerialParser::modserialCallback, MODSERIAL::RxAutoDetect);
	}

	char rx_buf_[256]; // TODO: tune buffer size

	// This method is called when a character goes into the TX buffer.
	void modserialCallback(MODSERIAL_IRQ_INFO *) {
		//comPC.rxBu
		comPC.move(rx_buf_, comPC.rxBufferGetCount());
		parseMsg(rx_buf_);
	}

};


int main()
{
	ModSerialParser serialParser;

	while (1) 
	{
		wait_ms(10);

		if (serialParser.comPC.readable())
		{

		}

		serialParser.setMsg(MsgType::Lspeed);
		serialParser.comPC.printf(serialParser.outgoingMsg_);

		serialParser.setMsg(MsgType::Rspeed);
		serialParser.comPC.printf(serialParser.outgoingMsg_);
	}
}
