from mot_cmd_py import *


def quickTest():

    ser = SerialParser()
    print('parseMsg', ser.parseMsg())
    ser.setMsg(b'k2ajksd')
    print('parseMsg', ser.parseMsg())
    print(ser.Aorder)
    ser.Aorder = 3.
    print(ser.Aorder)
    ser.Aorder = 2**32-1.
    ser.odom.a = 4
    ser.sendMsg(MsgType.Odom)
    ser.sendMsg(MsgType.Aorder)
    print(ser.getMsg())
    ser.Aorder = 77.
    print(ser.Aorder)
    ser.setMsg(ser.getMsg())
    print('parseMsg', ser.parseMsg())
    print('getdata', ser.Aorder)




if __name__ == "__main__":
    quickTest()

