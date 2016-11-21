#include "mbed.h"
#include <MODSERIAL.h>
#include "SerialParser.h"


DigitalOut led2(LED2);
 

// This function is called when a character goes into the RX buffer.
void rxCallbackFunc(MODSERIAL_IRQ_INFO *) {
	led2 = !led2;
}

struct ModSerialParser : public SerialParser<MODSERIAL>
{
	MODSERIAL comPC{ SERIAL_TX, SERIAL_RX, 1024 };

	ModSerialParser() : SerialParser<MODSERIAL>(comPC) 
	{
		comPC.baud(BAUD_RATE);
		
		comPC.autoDetectChar(delimiter_end_);
		comPC.attach(this, &ModSerialParser::messageEndCallback, MODSERIAL::RxAutoDetect);
		comPC.attach(this, &ModSerialParser::rxOverflowCallback, MODSERIAL::RxOvIrq);
		//comPC.attach(this, &ModSerialParser::rxCallback, MODSERIAL::RxIrq);
		//comPC.attach(&rxCallbackFunc, MODSERIAL::RxIrq);

		led2 = 1;
		printf("Started ModSerialParser \n");
	}

	int bytesReceived{ 0 };
	volatile int processing = 0;

	// This method is called when a character goes into the TX buffer.
	void rxOverflowCallback(MODSERIAL_IRQ_INFO *) {
		motorCmdData.msgCorrupted++;
		printf("overflow detected");
	}

	void rxCallback(MODSERIAL_IRQ_INFO *) {
		led2 = !led2;
		//printf("mbed rxCallback\n");
	}

	// This method is called when a character goes into the TX buffer.
	void messageEndCallback(MODSERIAL_IRQ_INFO *) {
		

		auto buf_size = rx_buffer_.size();
		auto buf_count = comPC.rxBufferGetCount();
		bytesReceived += buf_count;
		rx_buffer_.resize(buf_size + buf_count);
		comPC.move(&rx_buffer_[buf_size], buf_count);

		rx_buffer_.back() = delimiter_end_;

		auto msgType = parseMsg();

		if (MsgType::NONE != msgType)
		{
			//printf("received\n");
			if (msgType == MsgType::Status || motorCmdData.Status == ECHO_MODE) {
				sendMsg(msgType); // always echo status messages
			}

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

//Serial pc(USBTX, USBRX); // tx, rx

int main()
{
	ModSerialParser serialParser;

	auto& cmdData = serialParser.motorCmdData;
	//led2 = 0;
	Ticker odomTicker;
	//cmdData.Status = 1;
	cmdData.odomPeriod = 1000;
	//printf("TwistMsg %u\n", sizeof(TwistMsg));
	//printf("OdomMsg %u\n", sizeof(OdomMsg));

	while(true)
	{
		printf("cmdData.Status1 %d\n", cmdData.Status);
		
		while (serialParser.motorCmdData.Status == 0) {
			//serialParser.sendMsg(MsgType::Status);
			//led2 = !led2;
			wait_ms(2000);
		}
		
		printf("cmdData.Status %d\n", cmdData.Status);

		if(cmdData.Status != ModSerialParser::ECHO_MODE)
		{
			odomTicker.attach_us(&serialParser, &ModSerialParser::sendOdom, cmdData.odomPeriod*1000);
		}

		serialParser.sendMsg(MsgType::Status);

		while (cmdData.Status != 0){
			//wait_ms(50);
			// do servoing here
		}
		odomTicker.detach();

		wait_ms(5000);
		//serialParser.sendMsg(MsgType::bytesReceived);
		serialParser.sendMsg(MsgType::msgCorrupted);
		serialParser.sendMsg(MsgType::msgReceived);

		/*
		Sends an additional message (could be anything) to work around a Nucleo F401re bug : 
		Serial data is sometimes witheld on the card
		and gets transmitted only when the next message is sent.
		That's especially annoying when dealing with the last message of a sequence.
		Found a post with more details about it:
		https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Java/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fSTM32Java%2fNUCLEO%2dF401RE%20ST%2dLINKV2%2d1%20not%20reliably%20sending%20data%20to%20USB%20host&FolderCTID=0x01200200770978C69A1141439FE559EB459D758000F9A0E3A95BA69146A17C2E80209ADC21&currentviews=794
		*/
		serialParser.sendMsg(MsgType::Status);

		cmdData.msgReceived = 0;
		cmdData.bytesReceived = 0;
		cmdData.msgCorrupted = 0;
		
	}
}
