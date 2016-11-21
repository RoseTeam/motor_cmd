#include "SerialParser.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <thread>
#include <exception>

#include <stdlib.h>
#include <stdio.h>

#include "../RS-232/rs232.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


void print_hex(int c)
{
	std::ios::fmtflags f(std::cout.flags());

	auto cc = static_cast<uchar>(c);
	if (cc == '\'' || cc == '\n')
		std::cout << cc;
	else
		std::cout << "\\x" << std::setw(2) << std::setfill('0') << std::hex << (int)cc;

	std::cout.flags(f);
}

struct CSerialHelper
{
	int cport_nr;        /* 0 <-> /dev/ttyS0 (COM1 on windows) : starts with 0 */

	char mode[4] = { '8','N','1',0 };

	CSerialHelper(int port_num, int baud_rate) : cport_nr(port_num)
	{			

		for(int port = 0; port < 5; port++)
		{
			if (!RS232_OpenComport(port, baud_rate, mode))
			{
				cport_nr = port;
				std::cout << "using COM" << port+1 << " at " << baud_rate << " bps" << std::endl << std::endl;
				return;
			}
		}

		throw std::exception("Can not open comport");
	}

	void putc(int c)
	{
		RS232_SendByte(cport_nr, (char)c);
	}

	~CSerialHelper() {
		RS232_CloseComport(cport_nr);
	}
};

struct CSerialParser : public SerialParser<CSerialHelper>
{
	CSerialHelper ser_helper{0, BAUD_RATE};

	std::vector<uchar> buf_;

	int stats[static_cast<int>(MsgType::NONE)] = {0};
	bool print_rcvd_ = false;
	volatile bool _run_thread_flag{ true };

	CSerialParser() : 
		SerialParser<CSerialHelper>(ser_helper) 
	{
		buf_.resize(256,0);

		std::thread([&]() {

			while (_run_thread_flag)
			{
				int n = RS232_PollComport(ser_helper.cport_nr, &buf_[0], buf_.size());

				if (n > 0)
				{
					printf("received %i bytes: %s\n", n, (char *)&buf_[0]);

					for (int i = 0; i < n; i++)
					{
						receivec(buf_[i]);
					}
				}

				std::this_thread::yield();
			}

		}).detach();
	}

	~CSerialParser() { _run_thread_flag = false; }

	void receivec(int c)
	{
		//printf("char:{%c}",static_cast<char>(c));
		if (static_cast<char>(c) == delimiter_end_)
		{
			rx_buffer_.push_back(delimiter_end_);

			do
			{
				auto msgType = parseMsg();
				if (msgType == MsgType::NONE)
				{
					std::string msg((char*)&rx_buffer_[0], rx_buffer_.size());
					printf("can't parse; keeping %u bytes: %s\n", rx_buffer_.size(), msg.c_str());
					break;
				}
				else {
					stats[static_cast<int>(msgType)]++;

					if (print_rcvd_ || static_cast<int>(msgType) >= static_cast<int>(MsgType::Status))
					{
						std::cout << "recvd " << msgType << " ";
						motorCmdData.printVal(std::cout, msgType) << std::endl;
					}
					/*
					if (msgType == MsgType::Status)
						if (motorCmdData.Status == 0)
						{
							motorCmdData.Status = 1;
							sendMsg(MsgType::Status);
						}
					*/
				}

			} while (!rx_buffer_.empty());
		}
		else
		{
			rx_buffer_.push_back(c);
		}
	}

	void putc(int c)
	{
		print_hex(c);
		serial_helper_.putc(c);
		//receivec(c);
	}


	void print_stats() const
	{
		std::cout << std::endl << "====Stats====" << std::endl;
		for (int i = 0; i < static_cast<int>(MsgType::NONE); i++)
		{
			if (stats[i])
				std::cout << static_cast<MsgType>(i) << ": " << stats[i] << std::endl;
		}
	}
};

struct SerialHelperSender : public SerialParser<CSerialParser>
{
	CSerialParser rcv;
	SerialHelperSender() : SerialParser<CSerialParser>(rcv) {}
};

void print_packing()
{
	std::cout << typeid(MotorCmdData::Status).name() << " " << sizeof(MotorCmdData::Status) << std::endl;
	std::cout << typeid(MotorCmdData::Twist).name() << " " << sizeof(MotorCmdData::Twist) << std::endl;
	std::cout << typeid(MotorCmdData::Odom).name() << " " << sizeof(MotorCmdData::Odom) << std::endl;
}

struct TestSetup
{
	SerialHelperSender ser;

	void sendMsg(MsgType type)
	{ 
		std::cout << "Sending " << type << " ";
		ser.motorCmdData.printVal(std::cout, type) << std::endl;
		ser.sendMsg(type); 
	}

	~TestSetup() {
		if(!std::cin.peek())
			std::cout << "press Enter to stop" << std::endl;
		std::cin.get();

		ser.serial_helper_.print_stats();		
	}
};

void test_send()
{
	TestSetup setup;
	setup.ser.serial_helper_.print_rcvd_ = true;
	setup.ser.motorCmdData.odomPeriod = 1000;

	setup.sendMsg(MsgType::odomPeriod);
	setup.ser.motorCmdData.Status = SerialHelperSender::ECHO_MODE;
	setup.sendMsg(MsgType::Status);

	setup.sendMsg(MsgType::Twist);
	setup.sendMsg(MsgType::Odom);

	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void test_chrono(int N = 10)
{
	TestSetup setup;
	setup.ser.serial_helper_.print_rcvd_ = true;

	setup.ser.motorCmdData.odomPeriod = 600;
	setup.sendMsg(MsgType::odomPeriod);

	setup.ser.motorCmdData.Status = 1;
	setup.sendMsg(MsgType::Status);
	
	while (setup.ser.serial_helper_.stats[static_cast<int>(MsgType::Odom)] < N )//&& !std::cin.peek())
	{
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		std::this_thread::yield();
	}
	std::cout << "DONE: received" << N << "odom msgs" << std::endl;
	setup.ser.motorCmdData.Status = 0;
	setup.sendMsg(MsgType::Status);
}

void test_echo()
{

}

int main()
{
	int ret_code = 0;
	try{
		//test_send();
		test_chrono();
	}
	catch (std::exception const& exc)
	{
		std::cerr << "ERROR: " << exc.what();
		ret_code = 111;
	}

	std::cout << std::endl << "press Enter to exit" << std::endl;
	std::cin.get();

	return ret_code;
}

int main2()
{
	SerialHelperSender ser;

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