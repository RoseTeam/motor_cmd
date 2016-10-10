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
	SStatus,
	ENDMARKER
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
	SerialHelper& serial_helper_;

public:
	
	SerialParser(SerialHelper& helper) : serial_helper_(helper) {}

	static const uchar header_{ 0xff };
	static const uchar escaper_{ '\\' };
	static const uchar delimiter_{ '\n' };

	static constexpr int max_msg_size_ = 6;

	inline MsgData const& getDataRef(MsgType msgType) const { return data_[static_cast<uchar>(msgType)]; }
	inline MsgData & getDataRef(MsgType msgType) { return data_[static_cast<uchar>(msgType)]; }
	inline float const& getData(MsgType msgType) const { return data_[static_cast<uchar>(msgType)].f; }

	inline void setData(MsgType msgType, float const& val) { data_[static_cast<uchar>(msgType)].f = val; }

	bool parseMsg(uchar const* msg);

	void setMsg(MsgType msgType);
	inline void setPrepMsg(MsgType msgType, float const& val) { setData(msgType, val); setMsg(msgType); }

	inline uchar const* getMsg() const { return outgoingMsg_; }
	//inline char const* getMsg() const { return toto; }


	uchar outgoingMsg_[max_msg_size_]{ '\0' };

private:
	MsgData data_[static_cast<uchar>(MsgType::ENDMARKER)];
};

template<typename H>
bool SerialParser<H>::parseMsg(uchar const* msg)
{
	if (msg[0] >= static_cast<uchar>(MsgType::ENDMARKER))
	{
		return true; // first byte doesn't match a message type, ignore data
	}

	MsgType msgType = static_cast<MsgType>(msg[0]);
	//TODO 

	auto& data = getDataRef(msgType);
	std::memcpy(data.c, msg + 1, sizeof(data));
	return true;
}

template<typename H>
void SerialParser<H>::setMsg(MsgType msgType)
{
	outgoingMsg_[0] = static_cast<uchar>(msgType);

	MsgData const& data = getDataRef(msgType);
	std::memcpy(outgoingMsg_ + 1, data.c, 4);
	//outgoingMsg_[1] = 'a';
	//outgoingMsg_[2] = 'b';
	outgoingMsg_[5] = 0;
}
