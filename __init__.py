from mot_cmd_py import *


def quickTest():
    ser = SerialParser()
    print(ser.getData(MsgType.Aorder))
    ser.setData(MsgType.Aorder, 3.)
    print(ser.getData(MsgType.Aorder))
    print(2**32-1.)
    ser.setPrepMsg(MsgType.Aorder, 2**31-1.)
    print(ser.getData(MsgType.Aorder))
    print(ser.getMsg())
    ser.setData(MsgType.Aorder, 77.)
    print(ser.getData(MsgType.Aorder))
    print('len msg', len(ser.getMsg()))
    ser.parseMsg(ser.getMsg())
    print('getdata',ser.getData(MsgType.Aorder))
    return ser.getMsg()


if __name__ == "__main__":
    quickTest()

