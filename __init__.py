from mot_cmd_py import *

dummy_msg = b"'\x03\n'\x04\n'\x08\x00\x00\xceB\x00\x00\x00\x00\n'\x02\x9a\x99\x99@\n"

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
    #ser.sendMsg(MsgType.Odom)
    ser.sendMsg(MsgType.Aorder)
    msg = ser.getMsg(MsgType.Aorder)
    ser.Aorder = 77.
    print(ser.Aorder)
    ser.setMsg(msg)
    print('parseMsg', ser.parseMsg())
    print('getdata', ser.Aorder)

    ser.rxBuffer = dummy_msg

    print('parseMsg dummy', ser.parseMsg())
    print('parseMsg dummy', ser.parseMsg())
    print('parseMsg dummy', ser.parseMsg())

if __name__ == "__main__":
    quickTest()

