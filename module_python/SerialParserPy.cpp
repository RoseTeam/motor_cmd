#include <pybind11/pybind11.h>

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

using namespace pybind11;

#define DEF_READWRITE_FIELD(classname, name)\
	.def_readwrite(#name, &classname::name)

PYBIND11_PLUGIN(mot_cmd_py)
{
	module m("mot_cmd_py", "some docstring");


	enum_<MsgType>(m, "MsgType")
		.value("Xorder", MsgType::Xorder)
		.value("Yorder", MsgType::Yorder)
		.value("Aorder", MsgType::Aorder)
		.value("Lspeed", MsgType::Lspeed)
		.value("Rspeed", MsgType::Rspeed)
		.value("Twist", MsgType::Twist)
		.value("Odom", MsgType::Odom)
		.value("NONE", MsgType::None)
		;

	class_<TwistMsg>(m, "TwistMsg")
		.def(init<>())
		.def_readwrite("a", &TwistMsg::a)
		.def_readwrite("b", &TwistMsg::b)
		;

	class_<OdomMsg>(m, "OdomMsg")
		.def(init<>())
		DEF_READWRITE_FIELD(OdomMsg, a)
		DEF_READWRITE_FIELD(OdomMsg, b)
		.def("__str__",
			[](const OdomMsg &obj) {
				std::ostringstream str;
				str << "OdomMsg: " << obj.a << ", " << obj.b;
				return str.str();
		})
		;


	class_<SerialParser<SerialParserHelperPy>>(m, "SerialParserHelperPy");

	class_<SerialParserPy, SerialParser<SerialParserHelperPy>>(m, "SerialParser")
		.def(init<>())

		.def_readonly_static("delimiter_end_", &SerialParserPy::delimiter_end_)

		DEF_READWRITE_FIELD(SerialParserPy, Aorder)
		DEF_READWRITE_FIELD(SerialParserPy, Twist)
		DEF_READWRITE_FIELD(SerialParserPy, Odom)
		DEF_READWRITE_FIELD(SerialParserPy, Lspeed)
		DEF_READWRITE_FIELD(SerialParserPy, Rspeed)

		.def_property("rxBuffer", 
			[](SerialParserPy& ser) { return (bytes)std::string(&ser.rx_buffer_[0], ser.rx_buffer_.size());},
			[](SerialParserPy & ser, bytes const& data) {
					std::string strdata = data;
					ser.rx_buffer_.assign(strdata.begin(), strdata.end());
			})


		.def("sendMsg", &SerialParserPy::sendMsg)
		.def("parseMsg", &SerialParserPy::parseMsg)
		.def("setMsg", 
			[](SerialParserPy & ser, bytes const& data) {
				std::string strdata = data;
				ser.setMsg(strdata.c_str(), strdata.size()); 
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

