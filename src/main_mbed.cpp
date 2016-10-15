#include "mbed.h"
#include <MODSERIAL.h>
#include "SerialParser.h"


DigitalOut led2(LED2);
DigitalOut led3(LED3);

struct ModSerialParser : public SerialParser<MODSERIAL>
{
	MODSERIAL comPC{ SERIAL_TX, SERIAL_RX, 1024 };

	ModSerialParser() : SerialParser<MODSERIAL>(comPC) {
		comPC.baud(115200);

		comPC.autoDetectChar(delimiter_end_);
		comPC.attach(this, &ModSerialParser::messageEndCallback, MODSERIAL::RxAutoDetect);
	}

	int bytesReceived{ 0 };
	volatile int processing = 0;

	// This method is called when a character goes into the TX buffer.
	void rxOverflowCallback(MODSERIAL_IRQ_INFO *) {
		motorCmdData.msgCorrupted++;
	}

	// This method is called when a character goes into the TX buffer.
	void messageEndCallback(MODSERIAL_IRQ_INFO *) {
		
		auto buf_size = rx_buffer_.size();
		auto buf_count = comPC.rxBufferGetCount();
		bytesReceived += buf_count;
		rx_buffer_.resize(buf_size + buf_count);
		comPC.move(&rx_buffer_[buf_size], buf_count);

		auto msgType = parseMsg();

		if (MsgType::NONE != msgType)
		{
			if(msgType == MsgType::Status && motorCmdData.Status == 7)
				rx_buffer_.clear();
			//	led2 = !led2; // blink led when receiving a message
		}

	}

	void sendOdom()
	{
		motorCmdData.Odom.pos.x += 1;

		sendMsg(MsgType::Odom);
	}
};

int main()
{
	ModSerialParser serialParser;
	auto& cmdData = serialParser.motorCmdData;
	led2 = 0;
	Ticker odomTicker;

	while(true)
	{
		//serialParser.rx_buffer_.clear(); // that shouldn't be necessary

		while (cmdData.Status == 0)
			wait_ms(10);

		serialParser.sendMsg(MsgType::odomPeriod);

		if(cmdData.Status != ModSerialParser::ECHO_MODE)
			odomTicker.attach_us(&serialParser, &ModSerialParser::sendOdom, cmdData.odomPeriod*1000);

		while (cmdData.Status){
			//wait_ms(50);
			// do servoing here
		}
		odomTicker.detach();

		wait_ms(100);
		//serialParser.sendMsg(MsgType::bytesReceived);
		serialParser.sendMsg(MsgType::msgCorrupted);
		cmdData.msgReceived = 0;
		cmdData.msgCorrupted = 0;
		cmdData.Status = (uchar) serialParser.rx_buffer_.size();
		serialParser.sendMsg(MsgType::Status);
		cmdData.Status = 0;
		serialParser.sendMsg(MsgType::msgReceived);
	}
}
