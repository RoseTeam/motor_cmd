#pragma once

#include <cstdio>
#include <cstring>
#include <vector>

typedef unsigned char uchar;

union MsgData
{
	char c[5];
	int i = 0;
	float f;
};

enum class MsgType : uchar
{
	Xorder = 0,
	Yorder,
	Aorder,
	Lspeed,
	Rspeed,
	Ttwist,
	Vtwist,
	Twist,
	Odom,
	SStatus,
	Text,
	None // special value used as end marker, must remain in last position
};

struct TwistMsg
{
	float a{ 0 };
	float b{ 0 };
};

struct OdomMsg
{
	float a{ 0 };
	float b{ 0 };
};

struct Message
{
	MsgType type;
	MsgData data;
};

template<typename SerialHelper>
class SerialParser
{

public:

	SerialHelper& serial_helper_;

	SerialParser(SerialHelper& helper) : serial_helper_(helper) {}

	static const char header_{ '\'' };
	static const char delimiter_end_{ '\n' };
	
	void getDataRef(MsgType msgType, char* & obj, int& size);

	void setMsg(char const* msg, int size) { 
		for(int i = 0; i<size; i++)
			rx_buffer_.push_back(msg[i]);
	}

	MsgType parseMsg();
	void cleanUpBuffer(int index);

	void sendMsg(MsgType msgType);
	
	std::vector<char> rx_buffer_;

public:
	float Xorder;
	float Yorder;
	float Aorder{4.8f};
	float Lspeed;
	float Rspeed;
	float Ttwist;
	float Vtwist;
	int SStatus;

	OdomMsg odom;

	TwistMsg twist;
};

/**
 * @return true to signal existing buffer content have been processed 
 *		(or are unusable (corrupted, bad checksum)) and can be flushed
 *		false if message content is still incomplete and it should wait for more data.
 */
template<typename H>
MsgType SerialParser<H>::parseMsg()
{
	auto buf_size = rx_buffer_.size();
	if (buf_size < 4)
		return MsgType::None;

	
	if (rx_buffer_[0] != header_ || rx_buffer_[1] >= static_cast<uchar>(MsgType::None))
	{
		//return true; // first byte doesn't match a message type, ignore data
		rx_buffer_.clear();
		return MsgType::None;
	}

	
	MsgType msgType = static_cast<MsgType>(rx_buffer_[1]);

	if (msgType == MsgType::Text)
	{
		printf(&rx_buffer_[2]);
		rx_buffer_.clear();
		return MsgType::Text;
	}

	char* dataPtr;
	int dataLen;
	getDataRef(msgType, dataPtr, dataLen);

	if ((int)buf_size < dataLen + 2)
		return MsgType::None;

	if (dataPtr == nullptr || rx_buffer_[dataLen + 2] != delimiter_end_)
	{
		rx_buffer_.clear();
		return MsgType::None;
	}
	
	std::memcpy(dataPtr, &rx_buffer_[2], dataLen);
	
	cleanUpBuffer(dataLen+3);
	return msgType;
}

template<typename H>
void SerialParser<H>::cleanUpBuffer(int index)
{
	const int bufsize = rx_buffer_.size();
	int i = index;
	for (; i < bufsize; i++)
	{
		if (rx_buffer_[i] == header_)
		{
			break;
		}
	}

	for(int u = i; u<bufsize; u++)
	{
		rx_buffer_[u - i] = rx_buffer_[u];
	}

	rx_buffer_.resize(bufsize-i);
}

template<typename H>
void SerialParser<H>::sendMsg(MsgType msgType)
{
	char* dataPtr;
	int dataLen;
	getDataRef(msgType, dataPtr, dataLen);

	serial_helper_.putc(header_);

	serial_helper_.putc(static_cast<int>(msgType));

	for (int i = 0; i < dataLen; i++)
	{
		serial_helper_.putc(dataPtr[i]);
	}

	serial_helper_.putc(delimiter_end_);
}

template<typename H>
void SerialParser<H>::getDataRef(MsgType msgType, char* & obj, int& size)
{
	switch (msgType)
	{
	case MsgType::Aorder:
		obj = reinterpret_cast<char*>(&Aorder);
		size = sizeof(Aorder);
		break;
	case MsgType::Twist:
		obj = reinterpret_cast<char*>(&twist);
		size = sizeof(twist);
		break;
	case MsgType::Odom:
		obj = reinterpret_cast<char*>(&odom);
		size = sizeof(odom);
		break;

	default:
		obj = nullptr;
		size = 0;
	}
}