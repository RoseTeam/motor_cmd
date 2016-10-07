#include "SerialParser.h"

//#define DEBUG

#ifdef DEBUG
#define debug_print(...)  SerialParser::printf( ##__VA_ARGS__);
#else
#define debug_print(...)
#endif

int getFloat(char const* data, float *float_value) {
	uint32_t val;
	if (sscanf(data, "%X", &val) == 0) return 0;
	*float_value = *((float*)&val);
	return 1;
}

int getInt(char const* data, int *int_value) {
	uint32_t val;
	if (sscanf(data, "%x", &val) == 0) return 0;
	*int_value = val;
	return 1;
}

uint8_t computeCs(char const* buff, int len) {
	uint8_t cs = 0;
	for (int i = 0; i < len; i++) {
		cs += buff[i];
	}
	return cs;
}

bool SerialParser::verifyCs(char const* buff)
{
	int len = strlen(buff);
	//debug_print("len %d ",len);
	if (len < 3) return false;
	uint8_t readCs;
	//uint8_t bkpCmd = buff[2];
	//buff[2] = 0;
	if (sscanf(buff, "%hhx", &readCs) == 0) return false;
	//buff[2] = bkpCmd;
	uint8_t cs = 0;
	for (int i = 2; i < len; i++) {
		cs += buff[i];
	}
	//debug_print("cs %hhx rd %hhx\n",cs,readCs);
	return cs == readCs;
}

void SerialParser::printDebug(char const* message)
{
	BufferedSerial::printf("De%.30s\n", message);
}

void SerialParser::printTime(double msg)
{

	BufferedSerial::printf("De%f\n", msg);

}

void SerialParser::printServoing()
{
	BufferedSerial::printf("DeCT:%f,CL:%f,OT:%f,OL:%f,AT:%f,AL:%f\n", Ttwist, Vtwist, asser.getODO_SPEED_Theta(), asser.getODO_SPEED_Lin(), asser.orien, asser.dist);
}

void SerialParser::printOdo() {
	if (odoEnabled) {
		char buff[64];
		//float x = asser.ODO_X;
		float x = asser.ODO_X;
		float y = asser.ODO_Y;
		float t = asser.ODO_Theta;
		float s = asser.getODO_SPEED_Lin(); //asser.distanceCommand.GetSpeed();
		float w = asser.getODO_SPEED_Theta();//asser.angleCommand.GetSpeed();
		int len = sprintf(buff, "O%x;%x;%x;%x;%x\0", *((uint32_t *)&x), *((uint32_t *)&y), *((uint32_t *)&t), *((uint32_t *)&s), *((uint32_t *)&w));
		uint8_t cs = computeCs(buff, len);

		BufferedSerial::printf("%02X%s\n", cs, buff);

		/*char buff2[64];
		int uuid = asser.commandUUID;
		int running = asser.distanceCommand.isRunning() | asser.angleCommand.isRunning() ;
		float blockDist = asser.distanceCommand.getBlockageValue();
		float blockAngle = asser.angleCommand.getBlockageValue();
		int len2 = sprintf(buff2, "S%x;%x;%x;%x\0", uuid, running, *((uint32_t *)&blockDist), *((uint32_t *)&blockAngle));
		uint8_t cs2 = computeCs(buff2, len2);
		//SerialParser::printf("%02X%s\n",cs,buff);

		BufferedSerial::printf("%02X%s\n%02X%s\n",cs,buff,cs2,buff2);*/
	}
}

void SerialParser::resetComData()
{
	Ttwist = 0;
	Vtwist = 0;
}

//Function to be called when we have received the full packet
bool SerialParser::interpretData(char const* data)
{
	float a, b, c, d;
	if (data == NULL)
		return false;

	//debug_print("%s\n",data);
	if (!verifyCs(data)) {
		//debug_print("error cs\n");
		return true;
	}

	data += 3; // skip checksum + message type byte

	float incom_data;

	if (getFloat(data, &incom_data) == 0) {
		goto scanf_error;
	}

	switch (data[0])
	{
	case 'T':
		Ttwist = incom_data;
		debug_print("Ttwist %f\n", Ttwist);
		com_timeout.reset();
		break; //asser.angleCommand.SetSpeed(Ttwist);

	case 'V':
		Vtwist = incom_data; debug_print("Vtwist %f\n", Vtwist);
		com_timeout.reset();
		break; //asser.distanceCommand.SetSpeed(Vtwist);

	case 'S':
		SStatus = (int)incom_data;  //Reset
		asser.resetServoingParam();
		break;

	case 'U':
		UPower = (bool)incom_data;
		debug_print("DeUPower %d\n", UPower);
		asser.enable(UPower);
		resetComData();
		break;

	case '{':   KpPoLin = incom_data;
		data = strchr(data, ';');
		if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		KiPoLin = incom_data;
		data = strchr(data, ';'); if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		KdPoLin = incom_data;
		printf("Delin pid %f %f %f\n", KpPoLin, KiPoLin, KdPoLin);
		//asser.PidDistance.SetTunings(KpPoLin, KiPoLin, KdPoLin);
		break;

	case '(':   KpPoAng = incom_data;
		data = strchr(data, ';'); if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		KiPoAng = incom_data;
		data = strchr(data, ';'); if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		KdPoAng = incom_data;
		printf("Deang pid %f %f %f\n", KpPoAng, KiPoAng, KdPoAng);
		//asser.PidAngle.SetTunings(KpPoAng, KiPoAng, KdPoAng);
		break;

	case 'Z':   StartX = incom_data;
		data = strchr(data, ';'); if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		StartY = incom_data;
		data = strchr(data, ';'); if (data == NULL) goto scanf_error;
		data += 1;
		if (getFloat(data, &incom_data) == 0) goto scanf_error;
		StartTheta = incom_data;

		//asser.PidAngle.SetTunings(KpPoAng, KiPoAng, KdPoAng);

		resetComData();
		asser.resetServoingParam();
		printf("DeNew Coord %f %f %f\n", StartX, StartY, StartTheta);

		break;




	default: debug_print("error\n"); break;
	}
	goto end_parse;

scanf_error:
	debug_print("error scanf\n");

end_parse:
	releaseMessage();
	return true;

}
