import rssi_to_distance as rssi_dis
##import trilateration as tr
import localization as lx
from threading import Thread
import math
import json
import time
import sys
import os

# Disable printings
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore printings
def enablePrint():
    sys.stdout = sys.__stdout__

device_queue = {}

def calc_location(details, device_id):
    blockPrint()
    P=lx.Project(mode="2D",solver="LSE")
    for i in details:
        P.add_anchor(i[1], i[2])

    t,label=P.add_target()

    for i in details:
        t.add_measure(i[1], i[0])

    P.solve()
    enablePrint()
    device_queue[device_id]["location"] = t.loc
    print (t.loc)
    # return [t.loc.x, t.loc.y]

'''
temporary function. the coordinates should received from a data base
'''

def get_coords_receiver(receivers_MAC):
    x = 0
    y = 0
    if receivers_MAC == "24:6F:28:A9:83:C8":
        y = 0
        x = 5.1
    if receivers_MAC == "24:6F:28:A9:64:C8":
        y = 0
        x = 0
    if receivers_MAC == "24:6F:28:A9:87:40":
        y = 4.53
        x = 2.34
    return x,y

def localization_with_rssi(json_data):
    json_data = json.loads(json_data)
    device_id = json_data["dev_id"]
    sequence_number = json_data["seq_num"]
    receivers_MAC = json_data["MAC"]
    receiver_x, receiver_y = get_coords_receiver(receivers_MAC) ## collect data from database
    rssi = json_data["RSSI"]
    tx_pow = json_data["tx_pow"]
    distance = rssi_dis.rssi_to_dist(tx_pow, rssi, 9.25)
    
    if device_id not in device_queue:
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" :[]}
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
    else:
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_1"]) == 3:
                calc_location(device_queue[device_id]["s_1"], device_id)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_1"], device_id, ) )
                #thr.start()
                # print(location)
                device_queue[device_id]["S_2"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in data base
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_2"]) == 3:
                calc_location(device_queue[device_id]["s_2"], device_id)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_2"], device_id, ) )
                #thr.start()
                # print(location)
                device_queue[device_id]["S_1"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in database
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_3"]) == 3:
                calc_location(device_queue[device_id]["s_3"], device_id)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                #thr.start()
                # print(location)
                device_queue[device_id]["S_1"] = []
                device_queue[device_id]["S_2"] = []

                ## save location in data base


## test the code


import csv

print("Without threading")
# blockPrint()
start_time = time.time()
with open('Collected_Data/stationary.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        json_data = {"seq_num": int(row[0]), "dev_id": row[2], "tx_pow": -24, "RSSI": int(row[3]), "MAC": row[1], "acc_x": row[4], "acc_y": row[5], "acc_z": row[6], "gyro_x": row[7], "gyro_y": row[8], "gyro_z": row[9], "mag_x": row[10], "mag_y": row[11], "mag_z": row[12]}
        j_d = json.dumps(json_data)
        #thr = Thread(target=localization_with_rssi, args=(j_d,))
        #thr.start()
        localization_with_rssi(j_d)
stop_time = time.time()

# enablePrint()
print("--- %s seconds ---" % (time.time() - start_time))
