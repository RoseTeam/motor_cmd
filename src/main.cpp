#include "mbed.h"
#include "MODSERIAL/MODSERIAL.h"
#include "SerialParser.h"

struct SerialWrapper {

	MODSERIAL comPC{SERIAL_TX, SERIAL_RX};
	SerialParser serialParser;

	char rx_buf_[100];

	SerialWrapper()
	{
		comPC.baud(115200);

		comPC.autoDetectChar('\n');
		comPC.attach(this, &SerialWrapper::modserialCallback, MODSERIAL::RxAutoDetect);
	}

	// This method is called when a character goes into the TX buffer.
	void modserialCallback(MODSERIAL_IRQ_INFO *) {
		comPC.move(rx_buf_, 100);
		serialParser.parseMsg(rx_buf_);
	}

};


int main()
{
	SerialWrapper serialWrapper;

	while (1) 
	{
		wait_ms(10);
		serialWrapper.comPC.printf("%s",serialWrapper.serialParser.getData(MsgType::Lspeed));
		serialWrapper.comPC.printf("%s", serialWrapper.serialParser.getData(MsgType::Rspeed));
	}
}
