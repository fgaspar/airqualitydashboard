#! /usr/bin/env python3

import pigpio
import crcmod
import struct
import time
import os
import subprocess

class CommunicationTimeoutError(RuntimeError):
    pass

class CommunicationError(RuntimeError):
    pass

class Scd30 (object):
    def __init__(self, pigpio_addr='127.0.0.1', i2c_addr = 0x61, i2c_bus = 1, interval = 2, pressure = 1000):
        if os.geteuid() != 0:
            raise RuntimeError('SCD30: Must be run as root to modify I2C speed')
        i2c1_set_clkt_tout_path = os.path.join(os.path.dirname(__file__), 'scd30_auxbin', 'i2c1_set_clkt_tout')
        if subprocess.run([i2c1_set_clkt_tout_path, '20000'], capture_output=True, text=True).returncode != 0:
            raise RuntimeError('SCD30: Failed to set i2c clock')
        time.sleep(0.1)
        self._pi = pigpio.pi(pigpio_addr)
        if not self._pi.connected:
            raise CommunicationTimeoutError('SCD30: Failed to connecet to pigpio at "{}"'.format(pigpio_addr))
        self._h = self._pi.i2c_open(i2c_bus, i2c_addr)
        self._crcf = crcmod.mkCrcFun(0x131, 0xFF, False)

        time.sleep(0.1)

        if self.get_measurement_interval() != interval:
            self.set_measurement_interval(interval)
        time.sleep(0.1)
        if self.get_self_calibration() != 0:
            self.set_self_calibration(0)
        time.sleep(0.1)

        self.set_continuous_mode(pressure)
        time.sleep(0.1)

#TODO make something like this work
#    def __del__(self):
#        print(self._h)
#        self._pi.i2c_close(self._h)

    def read_data(self, interval = 0):
        time.sleep(interval)

        while not self._get_data_ready():
            time.sleep(0.2)

        self._pi.i2c_write_device(self._h, b'\x03\x00')
        bytes_expected = 18
        (count, data) = self._pi.i2c_read_device(self._h, bytes_expected)
        if count != bytes_expected:
            raise CommunicationError('SCD30: Did not receive expected number of bytes from data ready read')
        raw_data = self._data_check(data, bytes_expected)
        #co2, temperature, humidity
        return tuple(struct.unpack('>fff', raw_data))

    def _data_check(self, data, bytes_expected):
        if bytes_expected%3 != 0:
            raise RuntimeError('SCD30: "_data_check" caled with bytes not a multiple of 3')
        raw_data = bytearray()
        for i in range(0, bytes_expected, 3):
            local_crc = self._crcf(bytearray(data[i:i+2]))
            if data[i+2] != local_crc:
                raise CommunicationError('SCD30: CRC failed. Received data: {}, received crc {}, calculated crc {}'.format(data[i:i+2], data[i+2], local_crc))
            raw_data += data[i:i+2]
        return raw_data

    def _get_data_ready(self):
        self._pi.i2c_write_device(self._h, b'\x02\x02')
        bytes_expected = 3
        (count, data) = self._pi.i2c_read_device(self._h, bytes_expected)
        if count != bytes_expected:
            raise CommunicationError('SCD30: Did not receive expected number of bytes from data ready read')
        raw_data = self._data_check(data, bytes_expected)
        return bool(struct.unpack('>H', raw_data)[0])

    def get_measurement_interval(self):
        self._pi.i2c_write_device(self._h, b'\x46\x00')
        bytes_expected = 3
        (count, data) = self._pi.i2c_read_device(self._h, bytes_expected)
        if count != bytes_expected:
            raise CommunicationError('SCD30: Did not receive expected number of bytes from interval read')
        raw_data = self._data_check(data, bytes_expected)
        return struct.unpack('>H', raw_data)[0]

    def set_measurement_interval(self, interval, safe = True):
        cmd = b'\x46\x00'
        interval_b = struct.pack('>H', interval)
        crc_b =  struct.pack('>B', self._crcf(interval_b))
        data_crc = interval_b+crc_b
        self._pi.i2c_write_device(self._h, cmd+data_crc)
        if safe and (self.get_measurement_interval() != interval):
            RuntimeError('SCD30: Failed to set measurement interval')

    def get_self_calibration(self):
        self._pi.i2c_write_device(self._h, b'\x53\x06')
        bytes_expected = 3
        (count, data) = self._pi.i2c_read_device(self._h, bytes_expected)
        if count != bytes_expected:
            raise CommunicationError('SCD30: Did not receive expected number of bytes from interval read')
        raw_data = self._data_check(data, bytes_expected)
        return struct.unpack('>H', raw_data)[0]

    def set_self_calibration(self, value, safe = True):
        cmd = b'\x53\x06'
        value_to_write = 0
        if value:
            value_to_write = 1
        value_to_write_b = struct.pack('>H', value_to_write)
        crc_b =  struct.pack('>B', self._crcf(value_to_write_b))
        data_crc = value_to_write_b+crc_b
        self._pi.i2c_write_device(self._h, cmd+data_crc)
        if safe and (self.get_self_calibration() != value_to_write):
            RuntimeError('SCD30: Failed to set measurement interval')

    def set_continuous_mode(self, pressure):
        cmd = b'\x00\x10'
        pressure_b = struct.pack('>H', pressure)
        crc_b =  struct.pack('>B', self._crcf(pressure_b))
        data_crc = pressure_b+crc_b
        self._pi.i2c_write_device(self._h, cmd+data_crc)

    def stop_continuous_mode(self):
        cmd = b'\x01\x04'
        self._pi.i2c_write_device(self._h, cmd)

    def soft_reset(self):
        cmd = b'\xd3\x04'
        self._pi.i2c_write_device(self._h, cmd)

if __name__ == '__main__':
    scd30 = Scd30()
    # ctrl+c to close
    while True:
        data = scd30.read_data(interval = 5)
        print('''
C02 PPM : {} 
T degC  : {} 
Hum %rH : {} 
'''.format(*data))
        