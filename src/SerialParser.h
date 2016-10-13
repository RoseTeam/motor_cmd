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
		rx_buffer_.assign(msg, msg + size);
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

	OdomMsg Odom;

	TwistMsg Twist;
};

/**
 * @return true to signal existing buffer content have been processed 
 *		(or are unusable (corrupted, bad checksum)) and can be flushed
 *		false if message content is still incomplete and it should wait for more data.
 */
template<typename H>
MsgType SerialParser<H>::parseMsg()
{
	int buf_size = rx_buffer_.size();

	for (int first_idx = 0; first_idx < buf_size - 3; first_idx++, buf_size = rx_buffer_.size())
	{
		if (rx_buffer_[first_idx] != header_ ||
			rx_buffer_[first_idx+1] >= static_cast<uchar>(MsgType::None))
		{
			//return true; // first byte doesn't match a message type, ignore data
			continue;
		}

		MsgType msgType = static_cast<MsgType>(rx_buffer_[first_idx+1]);

		if (msgType == MsgType::Text)
		{
			printf(&rx_buffer_[first_idx+2]);
			rx_buffer_.clear();
			return MsgType::Text;
		}

		char* dataPtr;
		int dataLen;
		getDataRef(msgType, dataPtr, dataLen);

		if (buf_size < first_idx + dataLen + 2)
		{
			if(first_idx)
				cleanUpBuffer(first_idx);

			return MsgType::None;
		}

		if (dataPtr == nullptr)
			//|| rx_buffer_[first_idx + dataLen + 2] != delimiter_end_)
		{
			continue;
		}

		std::memcpy(dataPtr, &rx_buffer_[first_idx + 2], dataLen);

		cleanUpBuffer(first_idx + dataLen + 2);
		return msgType;
	}

	return MsgType::None;
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


#define CASE_SET_DATA_REF(msgType)\
	case MsgType::msgType:\
	obj = reinterpret_cast<char*>(&msgType);\
	size = sizeof(msgType);\
	break

template<typename H>
void SerialParser<H>::getDataRef(MsgType msgType, char* & obj, int& size)
{
	switch (msgType)
	{
	CASE_SET_DATA_REF(Aorder);
	CASE_SET_DATA_REF(Twist);
	CASE_SET_DATA_REF(Odom);
	CASE_SET_DATA_REF(Lspeed);
	CASE_SET_DATA_REF(Rspeed);

	default:
		obj = nullptr;
		size = 0;
	}
}