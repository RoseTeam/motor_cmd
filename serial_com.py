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

        self.bauds = SerialParser.BAUD_RATE
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
        self.msgSent = 0

        N_max = 200
        self.thread = Thread(target=self.receiving,
                             args=(N_max,))
        self.thread.start()

        motorData = self.parser.motorCmdData


        #motorData.Status = SerialParser.ECHO_MODE
        #self.send_msg(MsgType.Status, True)

        twist_period = 11
        odom_period = twist_period#110


        motorData.odomPeriod = odom_period
        self.send_msg(MsgType.odomPeriod, True)

        #motorData.Status = 0
        #self.send_msg(MsgType.Status, True)

        for _ in range(1):  # that loop shouldn't be necessary
            motorData.Status = 1
            self.send_msg(MsgType.Status, True)


        for _ in range(N_max):
            time.sleep(twist_period/1000.)
            self.send_msg(MsgType.Odom)


        print('Exit send')

    def receiving(self, N_max = 10):

        num_received = {}

        do_loop = True

        motorData = self.parser.motorCmdData

        t = time.process_time()

        doPrint = N_max <= 20

        num_rcv = 0

        while do_loop:

            bytes = self.ser.read(self.ser.inWaiting())
            if bytes:

                #print('recBytes', self.parser.rxBuffer, '+', bytes)
                num_set = self.parser.recvData(bytes)

                while self.parser.rxBuffer:

                    msgType = self.parser.parseMsg()

                    if msgType == MsgType.NONE:

                        if doPrint:
                            if bytes == self.parser.rxBuffer:
                                print('       msg NONE', bytes)
                            else:
                                print('       msg NONE', bytes, self.parser.rxBuffer)
                        break
                    else:

                        num_received[msgType] = num_received.get(msgType, 0)+1

                        if msgType == MsgType.Odom:
                            num_rcv += 1
                            motorData.Twist.pos.x = 2
                            # self.send_msg(MsgType.Twist)

                        if msgType != MsgType.Odom or doPrint:
                            print('   prsd', msgType, motorData.get_data(msgType))



                        if msgType == MsgType.msgReceived:
                            print()
                            elapsed_time = time.process_time() - t
                            print('       Elapsed time:', elapsed_time)
                            print('               freq: %.2f Hz'%(N_max / elapsed_time))
                            print('msgReceived on Mbed: {}/{} <<<<<'.format(motorData.msgReceived, self.msgSent))
                            print('msgReceived on PC  :')
                            for mType in num_received:
                                print('\t', mType, num_received[mType])

                        if msgType == MsgType.Status and motorData.Status == 0:
                            num_rcv = 0
                            #print (self.parser.rxBuffer, AllRcvBytes, bytes)
                            pass  # return

                        if num_rcv >= N_max:
                            if num_rcv == N_max:
                                print('**** N_max == {} detected'.format(N_max) )
                                motorData.Status = 0
                                self.send_msg(MsgType.Status, True)
                                #   self.send_msg(MsgType.Status, True)





    def send_msg(self, msgType, doPrint=False, sleep=.000):

        if sleep:
            time.sleep(sleep)

        self.msgSent += 1
        msg = self.parser.getMsg(msgType)

        if doPrint:
            print('sending', msgType, self.parser.motorCmdData.get_data(msgType), msg)

        # write hangs if passing longer msg (i.e. the length of a Twist)
        # why ??? split the msg to avoid that case
        self.ser.write(msg[:12])
        self.ser.write(msg[12:])

        # self.ser.write(msg)


    def __del__(self):
        if self.ser:
            self.ser.close()
        if self.thread:
            self.thread.join()


if __name__ == '__main__':

    s = SerialCom()
