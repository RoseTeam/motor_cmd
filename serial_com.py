"""
Lots of help from here:
http://stackoverflow.com/questions/1093598/pyserial-how-to-read-last-line-sent-from-serial-device
"""
from threading import Thread
import time
import serial
import collections
import time

from motor_cmd_py import SerialParser, MsgType

class SerialCom(object):

    def __init__(self, init=50):

        self.last_t = 0
        self.t_offset = 0

        self.bauds = 115200
        self.start_time = time.time()


        self.thread = None
        ports = [3, 4]

        for com_num in ports:
            try:
                self.port = "com%d"%com_num
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.bauds,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1,
                    xonxoff=0,
                    rtscts=0,
                    interCharTimeout=None
                )
                print("connected to", self.port)
                break
            except serial.serialutil.SerialException:
                #no serial connection
                #import traceback
                #traceback.print_exc()
                self.ser = None
        else:
            raise Exception("failed connecting to any com ports in {}".format(ports))

        self.parser = SerialParser()

        self.thread = Thread(target=self.receiving,
                             args=())
        self.thread.start()


    def receiving(self):

        num_received = {}

        do_loop = True

        motorData = self.parser.motorCmdData

        self.msgSent = 0

        def send_msg(msgType, doPrint=False):
            self.msgSent += 1
            msg = self.parser.getMsg(msgType)

            if doPrint:
                print('sending', msgType, msg)

            # write hangs if passing longer msg (i.e. the length of a Twist)
            # why ??? split the msg to avoid that case
            self.ser.write(msg[:12])
            self.ser.write(msg[12:])

            #self.ser.write(msg)

        t = time.process_time()

        motorData.Status = 1
        #motorData.Status = SerialParser.ECHO_MODE
        #send_msg(MsgType.msgReceived)
        motorData.odomPeriod = 10
        send_msg(MsgType.odomPeriod, True)

        #time.sleep(1)

        for i in 0,: #that loop shouldn't be necessary
            send_msg(MsgType.Status, True)


        for i in range(2):
            send_msg(MsgType.Odom, True)

        N_max = 3

        num_rcv = 0

        AllRcvBytes = b""


        while do_loop:

            bytes = self.ser.read(self.ser.inWaiting())
            #print(bytes)
            if bytes:
                AllRcvBytes += bytes
                num_set = self.parser.recvData(bytes)
                #print('setmsg', num_set, self.parser.rxBuffer)
                while True:
                    msgType = self.parser.parseMsg()

                    if msgType == MsgType.NONE:
                        print('                                             msg NONE')
                        break
                    else:
                        num_received[msgType] = num_received.get(msgType, 0)+1
                        if num_rcv >= N_max:
                            if num_rcv == N_max:
                                print('**** N_max == {} detected'.format(N_max) )
                                motorData.Status = 0
                                send_msg(MsgType.Status, True)
                                send_msg(MsgType.Status, True)

                        print('recvd', msgType, motorData.get_data(msgType))

                    if msgType == MsgType.msgReceived:

                        print('Elapsed time', time.process_time() - t)
                        print('msgReceived on Mbed:{}/{}'.format(motorData.msgReceived, self.msgSent))
                        print('msgReceived on server:')
                        for mType in num_received:
                            print('\t', mType, num_received[mType])


                    if msgType == MsgType.Odom:
                        num_rcv += 1
                        motorData.Twist.pos.x = 2
                        send_msg(MsgType.Twist)

                    if msgType == MsgType.Status and motorData.Status == 0:
                        return


    def __del__(self):
        if self.ser:
            self.ser.close()
        if self.thread:
            self.thread.join()


if __name__ == '__main__':

    s = SerialCom()
