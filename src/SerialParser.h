#pragma once

#include <cstdio>
#include <cstring>

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
	twist,
	SStatus,
	ENDMARKER
};

struct TwistMsg
{
	float a{ 0 };
	float b{ 0 };
};

template<MsgType msgType>
struct TypeTrait
{
	typedef float mType;
};

template<>
struct TypeTrait<MsgType::Xorder>
{
	typedef int mType;
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

	static const char header_{ '\xff' };
	static const char escaper_{ '\\' };
	static const char delimiter_{ '\n' };

	static constexpr int max_msg_size_ = 6;
	
	void getDataRef(MsgType msgType, char* & obj, int& size);

	//inline float const& getData(MsgType msgType) const { return data_[static_cast<uchar>(msgType)].f; }

	//inline void setData(MsgType msgType, float const& val) { data_[static_cast<uchar>(msgType)].f = val; }

	bool parseMsg(char const* msg);

	void sendMsg(MsgType msgType);
	

	//MsgData data_[static_cast<uchar>(MsgType::ENDMARKER)];

	float Xorder;
	float Yorder;
	float Aorder;
	float Lspeed;
	float Rspeed;
	float Ttwist;
	float Vtwist;
	int SStatus;

	TwistMsg twist;
};

/**
 * @return true to signal existing buffer content have been processed 
 *		(or are unusable (corrupted, bad checksum)) and can be flushed
 *		false if message content is still incomplete and it should wait for more data.
 */
template<typename H>
bool SerialParser<H>::parseMsg(char const* msg)
{
	if (msg[0] >= static_cast<uchar>(MsgType::ENDMARKER))
	{
		return true; // first byte doesn't match a message type, ignore data
	}

	MsgType msgType = static_cast<MsgType>(msg[0]);
	
	char* dataPtr;
	int dataLen;
	getDataRef(msgType, dataPtr, dataLen);
	std::memcpy(dataPtr, msg + 1, dataLen);
	return true;
}

template<typename H>
void SerialParser<H>::sendMsg(MsgType msgType)
{
	char* dataPtr;
	int dataLen;
	getDataRef(msgType, dataPtr, dataLen);

	serial_helper_.putc(static_cast<int>(msgType));

	for (int i = 0; i < dataLen; i++)
	{
		serial_helper_.putc(dataPtr[i]);
		if (dataPtr[i] == delimiter_)
		{
			serial_helper_.putc(delimiter_);
		}
	}
	serial_helper_.putc(delimiter_);
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
	case MsgType::twist:
		obj = reinterpret_cast<char*>(&twist);
		size = sizeof(twist);
		break;

	default:
		obj = nullptr;
		size = 0;
	}
}