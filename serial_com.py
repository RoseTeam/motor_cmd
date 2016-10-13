"""
Listen to serial, return most recent numeric values
Lots of help from here:
http://stackoverflow.com/questions/1093598/pyserial-how-to-read-last-line-sent-from-serial-device
"""
from threading import Thread
import time
import serial
import collections
import struct
import time

from mot_cmd_py import SerialParser, MsgType

MAX_SHORT = (1 << 16) - 1


class SerialData(object):

    def __init__(self, init=50):

        self.num_received_lines = 0
        self.received_lines = collections.deque()
        self.last_t = 0
        self.t_offset = 0

        self.bauds = 115200
        self.start_time = time.time()

        self.baud_meas_time = time.time()
        self.num_symbols = 0
        self.measured_bauds = 0

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
                import traceback
                traceback.print_exc()
                self.ser = None
        else:
            print("failed connecting to any com ports in {}".format(ports))

        self.parser = SerialParser()


        self.thread = Thread(target=self.receiving if self.ser is not None else self.receiving_test,
                             args=())
        self.thread.start()


    def incr_bauds(self, line):

        ts = time.time()

        self.num_symbols += len(line)
        self.num_symbols += 1 #endline

        if ts - self.baud_meas_time >= 1:
            bauds = self.num_symbols / (ts - self.baud_meas_time)
            if self.measured_bauds == 0:
                self.measured_bauds = bauds
            else:
                self.measured_bauds += bauds
                self.measured_bauds /= 2

            self.num_symbols = 0
            self.baud_meas_time = ts

    def receiving_test(self):

        while True:
            import math
            time.sleep(.1)
            t = (time.time() - self.start_time) * 1000
            t %= MAX_SHORT
            t = int(t)
            val = int(MAX_SHORT * (1 + math.cos(t / 500)) / 2)

            line = b"%c%c%c%c" % (t >> 8, t % 256, val >> 8, val % 256)
            self.received_lines.append(line)
            self.incr_bauds(line)

    def receiving(self):

        while True:

            bytes = self.ser.read(self.ser.inWaiting())
            if bytes:
                print('setmsg', self.parser.setMsg(bytes), self.parser.rxBuffer)
                #print('preparse', bytes)
                while True:
                    msgType = self.parser.parseMsg()
                    #print('postparse')
                    if msgType == MsgType.Odom:
                        #print(self.parser.odom.a)
                        self.parser.Aorder = self.parser.Odom.a
                        msg_out = self.parser.getMsg(MsgType.Aorder)
                        self.ser.write(msg_out)
                        #print("msg_out", msg_out)

                    if msgType == MsgType.NONE:
                        break

                    print('recvd', msgType, '' if msgType == MsgType.NONE else getattr(self.parser, str(msgType).split('.')[-1]) )
                    print()
    def __iter__(self):
        return self

    def __next__(self):

        while True:

            if not self.received_lines:
                raise StopIteration

            #return a float value or try a few times until we get one
            raw_line = self.received_lines.popleft()
            self.num_received_lines += 1

            try:
                st = struct.unpack('>HH', raw_line)
                t = st[0]
                # convert to [0, 1] interval from [0, MAX_SHORT]
                val = st[1] / MAX_SHORT
                #idx = st[2]

                if t < self.last_t:
                    self.t_offset += MAX_SHORT

                self.last_t = t

                t += self.t_offset

                #print(t, val)
                return t, val

            except struct.error:
                #not a number
                try:
                    print(raw_line.decode("utf-8"))
                except:
                    print(raw_line)

    def __del__(self):
        if self.ser:
            self.ser.close()
        if self.thread:
            self.thread.join()


if __name__ == '__main__':
    s = SerialData()

    while True:
        for val in s:
            if s.num_received_lines % 100 == 0:
                print(int(s.measured_bauds))
            #print(val)

