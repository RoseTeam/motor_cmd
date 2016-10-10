from mot_cmd_py import *


def quickTest():

    ser = SerialParser()

    print(ser.Aorder)
    ser.Aorder = 3.
    print(ser.Aorder)
    ser.Aorder = 2**32-1.
    ser.sendMsg(MsgType.Aorder)
    print(ser.getMsg())
    ser.Aorder = 77.
    print(ser.Aorder)
    ser.parseMsg(ser.getMsg()+b"laksjd")
    print('getdata', ser.Aorder)




if __name__ == "__main__":
    quickTest()

