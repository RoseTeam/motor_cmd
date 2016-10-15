#include <pybind11/pybind11.h>

#define NO_VOLATILE

#include "../src/SerialParser.h"
#include <sstream>

struct SerialParserHelperPy
{
	std::vector<char> outgoing_msg_;

	void putc(int c) {
		outgoing_msg_.push_back(static_cast<char>(c));
	}
};

struct SerialParserPy : public SerialParser<SerialParserHelperPy> {

	SerialParserHelperPy helper_;
	SerialParserPy() : SerialParser<SerialParserHelperPy>(helper_) {}
};

std::ostream& operator<<(std::ostream& str, Vect2d const& v)
{
	return str << v.x << ", " << v.y;
}

#define DEF_READWRITE_FIELD(classname, name)\
	.def_readwrite(#name, &classname::name)

#define ENUM_VALUE(enumname, name)\
	.value(#name, enumname::name)

using namespace pybind11;

PYBIND11_PLUGIN(_motor_cmd_py)
{
	module m("_motor_cmd_py", "some docstring");

	class_<Vect2d>(m, "Vect2d")
		.def(init<>())
		DEF_READWRITE_FIELD(Vect2d, x)
		DEF_READWRITE_FIELD(Vect2d, y)
		.def("__str__",
			[](const Vect2d &obj) {
				std::ostringstream str;
				str << obj;
				return str.str();
			})
		;

	class_<PIDCoeffs>(m, "PIDCoeffs")
		.def(init<>())
		DEF_READWRITE_FIELD(PIDCoeffs, kP)
		DEF_READWRITE_FIELD(PIDCoeffs, kI)
		DEF_READWRITE_FIELD(PIDCoeffs, kD)

		.def("__str__",
			[](PIDCoeffs const& obj) {
				std::ostringstream str;
				str << obj.kP << ", " << obj.kI << ", " << obj.kD;
				return str.str();
			})
		;

	class_<TwistMsg>(m, "TwistMsg")
		.def(init<>())
		DEF_READWRITE_FIELD(TwistMsg, pos)
		DEF_READWRITE_FIELD(TwistMsg, theta)

		.def("__str__", 
			[](TwistMsg const& obj) {
				std::ostringstream str;
				str << obj.pos << ", " << obj.theta;
				return str.str();
			})
		;

	class_<OdomMsg>(m, "OdomMsg")
		.def(init<>())
		DEF_READWRITE_FIELD(OdomMsg, pos)

		.def("__str__",
			[](const OdomMsg &obj) {
				std::ostringstream str;
				str << obj.pos;
				return str.str();
			})
		;

		enum_<MsgType>(m, "MsgType")
			ENUM_VALUE(MsgType, Twist)
			ENUM_VALUE(MsgType, Odom)
			ENUM_VALUE(MsgType, odomPeriod)
			ENUM_VALUE(MsgType, pidPropulsion)
			ENUM_VALUE(MsgType, pidDirection)
			ENUM_VALUE(MsgType, Status)
			ENUM_VALUE(MsgType, msgReceived)
			ENUM_VALUE(MsgType, msgCorrupted)
			ENUM_VALUE(MsgType, NONE)
			;

		class_<MotorCmdData>(m, "MotorCmdData")
			//.def(init<>())

			DEF_READWRITE_FIELD(MotorCmdData, Twist)
			DEF_READWRITE_FIELD(MotorCmdData, Odom)
			DEF_READWRITE_FIELD(MotorCmdData, odomPeriod)
			DEF_READWRITE_FIELD(MotorCmdData, pidPropulsion)
			DEF_READWRITE_FIELD(MotorCmdData, pidDirection)
			DEF_READWRITE_FIELD(MotorCmdData, Status)
			DEF_READWRITE_FIELD(MotorCmdData, msgReceived)
			DEF_READWRITE_FIELD(MotorCmdData, msgCorrupted)
			;

	class_<SerialParser<SerialParserHelperPy>>(m, "SerialParserHelperPy");

	class_<SerialParserPy, SerialParser<SerialParserHelperPy>>(m, "SerialParser")
		.def(init<>())

		.def_readonly_static("ECHO_MODE", &SerialParserPy::ECHO_MODE)
		.def_readonly_static("delimiter_end_", &SerialParserPy::delimiter_end_)

		DEF_READWRITE_FIELD(SerialParserPy, motorCmdData)

		.def_property("rxBuffer", 
			[](SerialParserPy& ser) { 
			if (ser.rx_buffer_.empty())
				return (bytes)"";
			return (bytes)std::string(&ser.rx_buffer_[0], ser.rx_buffer_.size());
			},
			[](SerialParserPy & ser, bytes const& data) {
					std::string strdata = data;
					ser.rx_buffer_.assign(strdata.begin(), strdata.end());
			})

		.def("sendMsg", &SerialParserPy::sendMsg)
		.def("parseMsg", &SerialParserPy::parseMsg)
		.def("recvData", 
			[](SerialParserPy & ser, bytes const& data) {
				std::string strdata = data;
				ser.recvData(strdata.c_str(), strdata.size()); 
				return strdata.size(); 
			})

		.def("getMsg", 
			[](SerialParserPy& ser, MsgType msgType) {
				ser.sendMsg(msgType);
				auto ret = (bytes)std::string(&ser.helper_.outgoing_msg_[0], ser.helper_.outgoing_msg_.size());
				ser.helper_.outgoing_msg_.clear();
				return ret;
			})
		;
		
	return m.ptr();
}

