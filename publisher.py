#! /usr/bin/env python3

import time
import threading
from sensors import pms5003, bme680, scd30
from influxdb import InfluxDBClient

DBG = False
PERIOD = 300

def use_db_dbg(db_client):
    db_name = 'sensors_dbg'
    if not db_name in [d['name'] for d in db_client.get_list_database()]:
        db_client.create_database(db_name)
        db_client.switch_database(db_name)
        db_client.create_retention_policy('short', '1h', '1', default=True)
    else:
        db_client.switch_database(db_name)



def use_db_main(db_client):
    db_name = 'sensors'
    if not db_name in [d['name'] for d in db_client.get_list_database()]:
        db_client.create_database(db_name)
        db_client.switch_database(db_name)
        db_client.create_retention_policy('long', '260w', '1', default=True)
    else:
        db_client.switch_database(db_name)


def push_data(db_client, sensor, fields, tags={}):
    json_body = [
        {
            "measurement": sensor,
            "tags": tags,
            "fields": fields
        }
    ]
    db_client.write_points(json_body)

    
def main():
    db_client = InfluxDBClient()
    db_lock = threading.Lock()
    if DBG:
        use_db_dbg(db_client)
    else:
        use_db_main(db_client)
    threading.Thread(target=push_pms5003, args=(db_client, db_lock, PERIOD,)).start()
    threading.Thread(target=push_bme680,  args=(db_client, db_lock, PERIOD,)).start()
    #threading.Thread(target=push_scd30,   args=(db_client, db_lock, PERIOD,)).start()

def push_pms5003(db_client, db_lock, period):
    sensor = pms5003.Pms5003()
    while True:
        data = sensor.read_data(interval = period)
        fields = {
            'PM1.0 ug/m3 (CF=1)': data[0], 
            'PM2.5 ug/m3 (CF=1)': data[1], 
            'PM10 ug/m3 (CF=1)': data[2], 
            'PM1.0 ug/m3 (atmos)': data[3], 
            'PM2.5 ug/m3 (atmos)': data[4], 
            'PM10 ug/m3 (atmos)': data[5], 
            '>0.3um in 0.1L air': data[6], 
            '>0.5um in 0.1L air': data[7], 
            '>1.0um in 0.1L air': data[8], 
            '>2.5um in 0.1L air': data[9], 
            '>5.0um in 0.1L air': data[10], 
            '>10um in 0.1L air': data[11]}
        with db_lock:
            push_data(db_client, 'pm5003', fields)

def push_bme680(db_client, db_lock, period):
    sensor = bme680.Bme680()
    while True:
        data = sensor.read_data(interval = period)
        fields = {
            'IAQ accuracy' : data[0],
            'IAQ'          : data[1],
            'T degC'       : data[2],
            'H %rH'        : data[3],
            'P hPa'        : data[4],
            'eCO2 ppm'     : data[5],
            'bVOCe ppm'    : data[6]}
        with db_lock:
            push_data(db_client, 'bme680', fields)

def push_scd30(db_client, db_lock, period):
    sensor = scd30.Scd30(interval = 10)
    while True:
        data = sensor.read_data(interval = period)
        fields = {
            'C02 PPM' : data[0],
            'T degC'  : data[1],
            'Hum %rH' : data[2]}
        with db_lock:
            push_data(db_client, 'scd30', fields)

if __name__ == '__main__':
    main()


        