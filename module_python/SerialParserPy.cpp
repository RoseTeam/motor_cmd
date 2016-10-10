#include <pybind11/pybind11.h>

#include "../src/SerialParser.h"


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

PYBIND11_PLUGIN(mot_cmd_py)
{
	module m("mot_cmd_py", "some docstring");


	enum_<MsgType>(m, "MsgType")
		.value("Xorder", MsgType::Xorder)
		.value("Yorder", MsgType::Yorder)
		.value("Aorder", MsgType::Aorder)
		.value("Lspeed", MsgType::Lspeed)
		.value("Rspeed", MsgType::Rspeed)
		;

	class_<TwistMsg>(m, "TwistMsg")
		.def(init<>())
		.def_readwrite("a", &TwistMsg::a)
		.def_readwrite("b", &TwistMsg::b)
		;
	
	class_<SerialParser<SerialParserHelperPy>>(m, "SerialParserHelperPy");

	class_<SerialParserPy, SerialParser<SerialParserHelperPy>>(m, "SerialParser")
		.def(init<>())
		.def_readwrite("Aorder", &SerialParserPy::Aorder)
		.def_readwrite("twist", &SerialParserPy::twist)

		.def("getMsg", [](SerialParserPy const& ser) {
				return (bytes)std::string(&ser.helper_.outgoing_msg_[0], ser.helper_.outgoing_msg_.size());
		})
		.def("sendMsg", &SerialParserPy::sendMsg)
		.def("parseMsg", &SerialParserPy::parseMsg)
		;

	return m.ptr();
}

