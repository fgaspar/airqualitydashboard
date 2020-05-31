#! /usr/bin/env python3

import subprocess
import threading
import re
import os
import time

BME_PATH = os.path.dirname(__file__)+'/../../bme680/bsec_bme680_linux/bsec_bme680'

class Bme680 (object):
    def __init__(self):
        self._data_lock = threading.Lock()
        self._ready = False
        self._iaq = 0
        self._iaq_acc = 0
        self._temperature = 0
        self._humidity = 0
        self._pressure = 0
        self._eco2 = 0
        self._bvoce = 0
        threading.Thread(target=self._update_data).start()
    
    def _update_data(self):
        p = subprocess.Popen(['./bsec_bme680'], cwd=os.path.dirname(__file__)+'/../../bme680/bsec_bme680_linux/', stdout=subprocess.PIPE, text=True)
        regex = re.compile(r'\((\d)\).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+).*?: ([\d.]+)')
        for line in p.stdout:
            result = regex.search(line)
            if result is None:
                raise RuntimeError('BME680: Line "{}" out of expected format'.format(line))
            with self._data_lock:
                self._ready = True
                self._iaq_acc = result[1]
                self._iaq = result[2]
                self._temperature = result[3]
                self._humidity = result[4]
                self._pressure = result[5]
                self._eco2 = result[8]
                self._bvoce = result[9]

    def read_data(self):
        ret = None
        while True:
            with self._data_lock:
                if self._ready:
                    ret = (int(self._iaq_acc),
                        float(self._iaq),
                        float(self._temperature),
                        float(self._humidity),
                        float(self._pressure),
                        float(self._eco2),
                        float(self._bvoce))
                    break
            time.sleep(1)
        return ret

if __name__ == '__main__':
    bme680 = Bme680()
    # ctrl+c to close
    while True:
        data = bme680.read_data()
        print('''
IAQ accuracy : {} 
IAQ          : {} 
T degC       : {} 
H %rH        : {} 
P hPa        : {} 
eCO2 ppm     : {} 
bVOCe ppm    : {} 
'''.format(*data))
        time.sleep(4)
        