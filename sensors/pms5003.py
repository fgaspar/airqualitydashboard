#! /usr/bin/env python3

import RPi.GPIO as GPIO
import serial
import struct
import time

PMS5003_SEQ_START = bytearray(b'\x42\x4d')

PMS5003_WARMUP_TIME = 60
PMS5003_MIN_PERIOD = 3


class CommunicationTimeoutError(RuntimeError):
    pass
class CheckError(RuntimeError):
    pass

class Pms5003 (object):
    def __init__(self, device='/dev/ttyAMA0', en_pin=22, reset_pin=27):
        self._serial = serial.Serial(device, baudrate=9600, timeout=10)
        self._en_pin = en_pin
        self._reset_pin = reset_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._en_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self._reset_pin, GPIO.OUT, initial=GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self._reset_pin, GPIO.LOW)
        self._serial.reset_input_buffer()
        time.sleep(0.1)
        GPIO.output(self._reset_pin, GPIO.HIGH)
        time.sleep(0.1)
        self._previous_read_time = None
        # #TODO use passive mode
        # #sleep: 42 4D E4 00 00 01 73
        # #wakeup: 42 4D E4 00 01 01 74
        # passive_cmd = PMS5003_SEQ_START + bytearray(b'\xe4\x00\x00')
        # check_local = sum(passive_cmd)
        # check_local %= 2**16
        # passive_cmd += struct.pack('>H',check_local)
        # #print('bla')
        # print(passive_cmd)
        # self._serial.reset_input_buffer()
        # self._serial.flushInput()
        # self._serial.write(passive_cmd)
        # print(self._serial.read(128))
        # print(self._serial.in_waiting)
        # exit(1)

    def _sensor_sleep(self):
        GPIO.output(self._en_pin, GPIO.LOW)
        time.sleep(0.1)
        self._serial.reset_input_buffer()

    def _sensor_wakeup(self):
        GPIO.output(self._en_pin, GPIO.HIGH)
        time.sleep(PMS5003_WARMUP_TIME)
        self._serial.reset_input_buffer()
    
    def read_data(self, interval = 0):
        if interval > PMS5003_WARMUP_TIME:
            time.sleep(interval - PMS5003_WARMUP_TIME)
            self._sensor_wakeup()
        else:
            time.sleep(interval)
            self._serial.reset_input_buffer()

        time_now = time.time()
        if self._previous_read_time is not None:
            elapsed_time = time_now - self._previous_read_time
            if elapsed_time < PMS5003_MIN_PERIOD:
                raise ValueError('PMS5003: Sensor must be read with a period of more than {}, {}s elapsed since last read'.format(PMS5003_MIN_PERIOD, elapsed_time))
        self._previous_read_time = time_now
        # Wait for start of packet
        packet_start = self._serial.read_until(PMS5003_SEQ_START, size=100)
        if len(packet_start) < 2 or packet_start[-2:] != PMS5003_SEQ_START:
            raise CommunicationTimeoutError("PMS5003: Could not find sequence start")
        # Initiate check (start of packet already found)
        check_local = sum(PMS5003_SEQ_START)
        # Read packet frame lenght
        frame_length_raw = bytearray(self._serial.read(2))
        if len(frame_length_raw) != 2:
            raise CommunicationTimeoutError("PMS5003: Did not receive bytes indicating length of packet to read")
        check_local += sum(frame_length_raw)
        frame_length = struct.unpack(">H", frame_length_raw)[0]

        data_raw = bytearray(self._serial.read(frame_length))
        if len(data_raw) != frame_length:
            raise CommunicationTimeoutError("PMS5003: Invalid frame length. Received {} out of {} expected bytes.".format(len(data_raw), frame_length))
        packet = struct.unpack(">HHHHHHHHHHHHHH", data_raw)
        data = packet[:13]
        check_received = packet[13]

        # Check doesn't incldue own bits
        check_local += sum(data_raw[:-2])
        check_local %= 2**16

        if check_received != check_local:
            raise CheckError("PMS5003: Check Mismatch. Got 0x{:02x} expected: 0x{:02x}, full packet: {}".format(check_received, check_local, packet))
        
        if interval > PMS5003_WARMUP_TIME:
            self._sensor_sleep()

        #Returns Tuple:
        #PM1.0 ug/m3 (CF=1),
        #PM2.5 ug/m3 (CF=1),
        #PM10 ug/m3 (CF=1),
        #PM1.0 ug/m3 (atmos),
        #PM2.5 ug/m3 (atmos),
        #PM10 ug/m3 (atmos),
        #>0.3um in 0.1L air,
        #>0.5um in 0.1L air,
        #>1.0um in 0.1L air,
        #>2.5um in 0.1L air,
        #>5.0um in 0.1L air,
        #>10um in 0.1L air
        return tuple(data[:-1]) # Don't return reserved data

if __name__ == '__main__':
    pms5003 = Pms5003()
    # ctrl+c to close
    while True:
        data = pms5003.read_data(interval = 90)
        print('''
PM1.0 ug/m3 (CF=1)  : {} 
PM2.5 ug/m3 (CF=1)  : {} 
PM10 ug/m3 (CF=1)   : {} 
PM1.0 ug/m3 (atmos) : {} 
PM2.5 ug/m3 (atmos) : {} 
PM10 ug/m3 (atmos)  : {} 
>0.3um in 0.1L air  : {} 
>0.5um in 0.1L air  : {} 
>1.0um in 0.1L air  : {} 
>2.5um in 0.1L air  : {} 
>5.0um in 0.1L air  : {} 
>10um in 0.1L air   : {}
'''.format(*data))        