#include "serial_helper.h"
#include "../RS-232/rs232.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <exception>

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


CSerialHelper::CSerialHelper(int port_num, int baud_rate) : cport_nr(port_num)
{

	for (int port = 0; port < 5; port++)
	{
		if (!RS232_OpenComport(port, baud_rate, mode))
		{
			cport_nr = port;
			std::cout << "using COM" << port + 1 << " at " << baud_rate << " bps" << std::endl << std::endl;
			return;
		}
	}

	throw std::exception("Can not open comport");
}

void CSerialHelper::putc(int c)
{
	RS232_SendByte(cport_nr, (char)c);
}

CSerialHelper::~CSerialHelper() {
	RS232_CloseComport(cport_nr);
}



CSerialParser::CSerialParser() :
	SerialParser<CSerialHelper>(ser_helper)
{
	buf_.resize(256, 0);

	std::thread([&]() {

		while (_run_thread_flag)
		{
			int n = RS232_PollComport(ser_helper.cport_nr, &buf_[0], buf_.size());

			if (n > 0)
			{
				//printf("received %i bytes: %s\n", n, (char *)&buf_[0]);

				for (int i = 0; i < n; i++)
				{
					receivec(buf_[i]);
				}
			}

			std::this_thread::yield();
		}

	}).detach();
}

void CSerialParser::receivec(int c)
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
				//printf("can't parse; keeping %u bytes: %s\n", rx_buffer_.size(), msg.c_str());
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

void CSerialParser::print_stats() const
{
	std::cout << std::endl << "====Stats====" << std::endl;
	for (int i = 0; i < static_cast<int>(MsgType::NONE); i++)
	{
		if (stats[i])
			std::cout << static_cast<MsgType>(i) << ": " << stats[i] << std::endl;
	}
}
