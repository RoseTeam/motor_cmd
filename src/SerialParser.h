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
	static const int BAUD_RATE{ 115200 };

	// struct holding data to be sync'ed accross serial connexion.
	VOLATILE MotorCmdData motorCmdData;
	std::vector<char> rx_buffer_;

	SerialHelper& serial_helper_;

	SerialParser(SerialHelper& helper) : serial_helper_(helper) {}

	static const uchar ECHO_MODE{ 2 }; // echo mode for testing : resend everything you receive
	static const char delimiter_end_{ '\n' };
	static const char HEADER_CHAR{ '/' };

	inline void recvData(char const* msg, int size) { 
		rx_buffer_.insert(rx_buffer_.end(), msg, msg + size);
	}

	MsgType parseMsg();

	/* returns true on success */
	bool sendMsg(MsgType msgType);

private:
	void cleanUpBuffer(int index);
};


#define PRINTF_DEBUG(...) //printf(__VA_ARGS__)

/**
 * @return true to signal existing buffer content have been processed 
 *		(or are unusable (corrupted, bad checksum)) and can be flushed
 *		false if message content is still incomplete and it should wait for more data.
 */
template<typename H>
MsgType SerialParser<H>::parseMsg()
{
	int buf_size = rx_buffer_.size();
	
	int first_idx = 0;
	for (; first_idx <= buf_size - 3; first_idx++)
	{
		if (//rx_buffer_[first_idx] != HEADER_CHAR ||
			 rx_buffer_[first_idx] >= static_cast<uchar>(MsgType::NONE)
			)
		{
			// first byte doesn't match a message type, 
			// skip that byte
			PRINTF_DEBUG("skipping %c\n", rx_buffer_[first_idx]);

			continue;
		}

		MsgType msgType = static_cast<MsgType>(rx_buffer_[first_idx]);

		char* dataPtr;
		int dataLen;
		motorCmdData.getDataRef(msgType, dataPtr, dataLen);

		if (dataPtr == nullptr)
			//|| rx_buffer_[first_idx + dataLen + 2] != delimiter_end_)
		{
			PRINTF_DEBUG("no match %c\n", msgType);

			// no matching message found, skip that byte.
			continue;
		}

		if (buf_size - first_idx < dataLen + 3)
		{
			PRINTF_DEBUG("waiting for more\n");
			//for (auto c : rx_buffer_){ PRINTF_DEBUG("%c", c);}

			// the buffer is shorter than expected message size,
			// wait until we receive more data
			break;
		}

		if (rx_buffer_[first_idx + dataLen + 2] != delimiter_end_)
		{
			PRINTF_DEBUG("no end marker %d %c %d\n", first_idx, rx_buffer_[first_idx], buf_size);
			//for (auto c : rx_buffer_) { PRINTF_DEBUG("%c", c); }

			continue;
		}

		uchar checkSum = rx_buffer_[first_idx];
		for (int j = first_idx+1; j < first_idx+1+dataLen; j++)
			checkSum += rx_buffer_[j];

		if (checkSum != static_cast<uchar>(rx_buffer_[first_idx + dataLen + 1]))
		{
			// check sum mismatch
			PRINTF_DEBUG("checksum mismatch\n");
			continue;
		}

		// copy message data to the destination variable
		std::memcpy(dataPtr, &rx_buffer_[first_idx + 1], dataLen);

		// removed bytes that were just read
		cleanUpBuffer(first_idx + 3 + dataLen);

		motorCmdData.msgReceived++;

		//printf("found\n");

		return msgType;
	}

	if (first_idx)
		cleanUpBuffer(first_idx);

	return MsgType::NONE;
}

template<typename H>
void SerialParser<H>::cleanUpBuffer(int index)
{
	const int bufsize = rx_buffer_.size();

	for(int u = index; u<bufsize; u++)
	{
		rx_buffer_[u - index] = rx_buffer_[u];
	}

	rx_buffer_.resize(bufsize-index);
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

	//serial_helper_.putc(HEADER_CHAR);

	uchar checkSum = static_cast<int>(msgType);
 
	serial_helper_.putc(checkSum);
	
	for (int i = 0; i < dataLen; i++)
	{
		checkSum += dataPtr[i];
		serial_helper_.putc(dataPtr[i]);
	}

	serial_helper_.putc(checkSum);

	serial_helper_.putc(delimiter_end_);

	return true;
}
