#include <vector>
#include <iostream>
#include <thread>
#include <exception>

#include <chrono>
#include <stdlib.h>
#include <stdio.h>
#include "serial_helper.h"


#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std::chrono;

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

	void sendMsg(MsgType type, bool print=true)
	{ 
		if(print){
			std::cout << "Sending " << type << " ";
			ser.motorCmdData.printVal(std::cout, type) << std::endl;
		}
		ser.sendMsg(type); 
	}

	~TestSetup() {

		std::cout << std::endl << "press Enter to continue" << std::endl;
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

void sendTwistRecOdom(int N = 10, int odomPeriodMs = 1, int twistPeriodMs = 100)
{
	TestSetup setup;
	setup.ser.serial_helper_.print_rcvd_ = N <= 10;

	setup.ser.motorCmdData.odomPeriod = odomPeriodMs;
	setup.sendMsg(MsgType::odomPeriod);

	setup.ser.motorCmdData.Status = 1;
	setup.sendMsg(MsgType::Status);
	
	auto t1 = high_resolution_clock::now();

	while (setup.ser.serial_helper_.stats[static_cast<int>(MsgType::Odom)] < N )//&& !std::cin.peek())
	{
		if (twistPeriodMs >= 0){
			setup.sendMsg(MsgType::Twist, false);
		}

		if(twistPeriodMs > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(twistPeriodMs));
		else
			std::this_thread::yield();
	}

	auto t2 = high_resolution_clock::now();

	auto duration = duration_cast<microseconds>(t2 - t1).count();

	std::cout << std::endl << "			DONE: received " << N << " odom msgs";
	std::cout << " in " << duration / 1.e6 << "s at " << N * 1e6 / duration << " msgs per s" << std::endl;

	setup.ser.motorCmdData.Status = 0;
	setup.sendMsg(MsgType::Status);
}

int main()
{
	int ret_code = 0;
	try{
		//test_send();
		sendTwistRecOdom(2000,5,5);
	}
	catch (std::exception const& exc)
	{
		std::cerr << "ERROR: " << exc.what();
		ret_code = 111;
	}
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