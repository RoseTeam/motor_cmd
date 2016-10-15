#include "SerialParser.h"
#include <vector>
#include <iomanip>
#include <iostream>

void _putc(int c)
{
	std::ios::fmtflags f(std::cout.flags());

	char cc = static_cast<char>(c);
	if (cc == '\'' || cc == '\n')
		std::cout << cc;
	else
		std::cout << "\\x" << std::setw(2) << std::setfill('0') << std::hex << c;

	std::cout.flags(f);
}

struct SerialHelper
{
	void putc(int c)
	{
		_putc(c);
	}
};

struct SerialReceiver : public SerialParser<SerialHelper>
{
	SerialHelper ser_helper;
	SerialReceiver() :SerialParser<SerialHelper>(ser_helper) {}

	std::vector<char> rxMsg_;
	void receivec(int c)
	{
		//printf("char:{%c}",static_cast<char>(c));
		if (static_cast<char>(c) == delimiter_end_)
		{
			//auto buf_size = rx_buffer_.size();
			//auto buf_count = rxMsg_.size();
			//rx_buffer_.resize(buf_size + buf_count);
			for (auto const& cc : rxMsg_)
				rx_buffer_.push_back(cc);
			
			rxMsg_.clear();

			do
			{
				auto msgType = parseMsg();
				if (msgType == MsgType::NONE)
				{
					std::string msg((char*)&rx_buffer_[0], rx_buffer_.size());
					printf("can't parse: %s\n", msg.c_str());
					break;
				}

				//printf("recvd: %d\n", motorCmdData.odomPeriod);

			} while (!rx_buffer_.empty());

		}
		else
		{
			rxMsg_.push_back(c);
		}
	}

	void putc(int c)
	{
		_putc(c);
		receivec(c);
	}
};

struct SerialHelperSender : public SerialParser<SerialReceiver>
{
	SerialReceiver rcv;
	SerialHelperSender() : SerialParser<SerialReceiver>(rcv) {}
};


SerialHelperSender ser;

void test_send()
{
	ser.motorCmdData.odomPeriod = 1000;
	ser.sendMsg(MsgType::odomPeriod);
	ser.motorCmdData.Status = SerialHelperSender::ECHO_MODE;
	ser.sendMsg(MsgType::Status);

	ser.sendMsg(MsgType::Odom);
	ser.sendMsg(MsgType::Odom);
}

int main()
{
	test_send();
	return 0;
}

int main2()
{
	char const* odomPeriod = "'\x02\xe8\x03\x00\x00\n";
	char const* Status = "'\x05\x02\n";
	char const* odom = "'\x01\x00\x00\x00\x00\x00\x00\x00\x00\n";


	char const* msg = "'\x03\n'\x04\n'\x08\x00\x00\xce""B\x00\x00\x00\x00\n'\x02\x9a\x99\x99""@\n";
	ser.recvData(msg, 24);
	ser.parseMsg();
	ser.parseMsg();

	auto& cmdData = ser.motorCmdData;

	const int N = 10;

	for(int i = 0; i<N; i++)
	{
		cmdData.odomPeriod = 0;
		printf("sending %d\n", cmdData.odomPeriod);
		ser.sendMsg(MsgType::odomPeriod);

		cmdData.odomPeriod = 20;
		printf("sending %d\n", cmdData.odomPeriod);
		ser.sendMsg(MsgType::odomPeriod);

		//ser.recvData(&ser.serial_helper_.rxMsg_[0], ser.serial_helper_.rxMsg_.size());
		//ser.parseMsg();

		printf("\n\n");
		ser.serial_helper_.putc('\'');
	}
	return 0;
}