#! /usr/bin/env python3

import time
import threading
from sensors import pm5003
from influxdb import InfluxDBClient

DBG = True
PERIOD = 5

def use_db_dbg(db_client):
    db_name = 'sensors_dbg'
    if not db_name in [d['name'] for d in db_client.get_list_database()]:
        db_client.create_database(db_name)
        db_client.create_retention_policy('short', '1h', '1', default=True)
    db_client.switch_database(db_name)

def use_db_main(db_client):
    db_name = 'sensors'
    if not db_name in [d['name'] for d in db_client.get_list_database()]:
        db_client.create_database(db_name)
        db_client.create_retention_policy('long', '5y', '1', default=True)
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
    threading.Thread(target=push_pm5003, args=(db_client, db_lock, PERIOD,)).start()

def push_pm5003(db_client, db_lock, period):
    sensor = pm5003.Pm5003()
    while True:
        data = sensor.read_data()
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
        time.sleep(period)


if __name__ == '__main__':
    main()


        