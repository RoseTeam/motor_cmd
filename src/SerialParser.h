#pragma once

#include <cstdio>
#include <cstring>
#include <vector>
#include "motor_cmd_data.h"

#if defined (NO_VOLATILE)
#define VOLATILE
#else
#define VOLATILE volatile
#endif

template<typename SerialHelper>
struct SerialParser
{
	// struct holding data to be sync'ed accross serial connexion.
	VOLATILE MotorCmdData motorCmdData;
	std::vector<char> rx_buffer_;

	SerialHelper& serial_helper_;

	SerialParser(SerialHelper& helper) : serial_helper_(helper) {}

	static const uchar ECHO_MODE{ 2 }; // echo mode for testing : resend everything you receive

	static const char header_{ '/' };
	static const char delimiter_end_{ '\n' };
	
	void recvData(char const* msg, int size) { 
		rx_buffer_.assign(msg, msg + size);
	}

	MsgType parseMsg();

	/* returns true on success */
	bool sendMsg(MsgType msgType);

private:
	void cleanUpBuffer(int index);
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

	for (int first_idx = 0; first_idx <= buf_size - 3; first_idx++, buf_size = rx_buffer_.size())
	{
		if (rx_buffer_[first_idx] != header_ ||
			rx_buffer_[first_idx+1] >= static_cast<uchar>(MsgType::NONE))
		{
			// first byte doesn't match a message type, 
			// skip that byte
			continue;
		}

		MsgType msgType = static_cast<MsgType>(rx_buffer_[first_idx+1]);

		char* dataPtr;
		int dataLen;
		motorCmdData.getDataRef(msgType, dataPtr, dataLen);

		if (dataPtr == nullptr)
			//|| rx_buffer_[first_idx + dataLen + 2] != delimiter_end_)
		{
			// no matching message found, skip that byte.
			continue;
		}

		if (buf_size < first_idx + dataLen + 2)
		{
			if(first_idx)
				cleanUpBuffer(first_idx);

			// the buffer is shorter than expected message size,
			// wait until we receive more data
			return MsgType::NONE;
		}

		// copy message data to the destination variable
		std::memcpy(dataPtr, &rx_buffer_[first_idx + 2], dataLen);

		// removed bytes that were just read
		cleanUpBuffer(first_idx + dataLen + 2);

		motorCmdData.msgReceived++;

		if (motorCmdData.Status == ECHO_MODE) {
			sendMsg(msgType);
		}

		return msgType;
	}

	return MsgType::NONE;
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
bool SerialParser<H>::sendMsg(MsgType msgType)
{
	char* dataPtr;
	int dataLen;
	motorCmdData.getDataRef(msgType, dataPtr, dataLen);

	if (dataPtr == nullptr){
		// error: msg type doesn't match a variable
		// check the mapping
		return false; 
	}
	serial_helper_.putc(header_);

	serial_helper_.putc(static_cast<int>(msgType));
	
	for (int i = 0; i < dataLen; i++)
	{
		serial_helper_.putc((uchar)dataPtr[i]);
	}

	serial_helper_.putc(delimiter_end_);

	return true;
}
