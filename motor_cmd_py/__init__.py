from _motor_cmd_py import *

def get_data(motorData, msgType):
    return None if msgType == MsgType.NONE else getattr(motorData, str(msgType).split('.')[-1])

dummy_msg = b"'\x03\n'\x04\n'\x08\x00\x00\xceB\x00\x00\x00\x00\n'\x02\x9a\x99\x99@\n"

twist_msg = b"'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\n"

MotorCmdData.get_data = get_data

def quickTest():

    o = OdomMsg();
    print('Odom:', o)
    v = Vect2d()
    print('Vect2d:', v)

    ser = SerialParser()
    print('parseMsg', ser.parseMsg())

    motorData = ser.motorCmdData

    ser.recvData(b'k2ajksd')
    print('parseMsg', ser.parseMsg())
    print(motorData.Status)
    motorData.Status = 3
    print(motorData.Status)
    motorData.Status = 1
    motorData.Odom.pos.x = 4
    #ser.sendMsg(MsgType.Odom)
    ser.sendMsg(MsgType.Status)
    msg = ser.getMsg(MsgType.Status)
    motorData.Status = 77
    print(motorData.Status)
    ser.recvData(msg)
    print('parseMsg', ser.parseMsg())
    print('getdata', motorData.Status)

    ser.rxBuffer = twist_msg

    while True:
        msgType = ser.parseMsg()
        if msgType == MsgType.NONE:
            break
        print('parseMsg:', msgType, motorData.get_data(msgType))

if __name__ == "__main__":
    quickTest()

