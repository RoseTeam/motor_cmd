#pragma once

#include "SerialParser.h"

void print_hex(int c);

struct CSerialHelper
{
	int cport_nr;        /* 0 <-> /dev/ttyS0 (COM1 on windows) : starts with 0 */

	char mode[4] = { '8','N','1',0 };

	CSerialHelper(int port_num, int baud_rate);

	void putc(int c);

	~CSerialHelper();
};

struct CSerialParser : public SerialParser<CSerialHelper>
{
	CSerialHelper ser_helper{ 0, BAUD_RATE };

	std::vector<uchar> buf_;

	int stats[static_cast<int>(MsgType::NONE)] = { 0 };
	bool print_rcvd_ = false;
	volatile bool _run_thread_flag{ true };

	CSerialParser();
	~CSerialParser() { _run_thread_flag = false; }

	void receivec(int c);
	void putc(int c)
	{
		//print_hex(c);
		serial_helper_.putc(c);
		//receivec(c);
	}

	void print_stats() const;
};
