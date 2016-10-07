#pragma once

#include <cstdio>

typedef unsigned char uchar;

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

class SerialParser
{
	float data_[static_cast<uchar>(MsgType::ENDMARKER)];
	
	char outgoingMsg_[100]{0};

public:

	inline float & getData(MsgType msgType) { return data_[static_cast<uchar>(msgType)]; }
	inline float const& getData(MsgType msgType) const { return data_[static_cast<uchar>(msgType)]; }
	inline void setData(MsgType msgType, float const& val) { data_[static_cast<uchar>(msgType)] = val; }

	bool parseMsg(char const* msg);

	void setMsg(MsgType msgType);
};
