import rssi_to_distance as rssi_dis
##import trilateration as tr
import localization as lx
##import thread
import math
import json
import time

device_queue = {}

def calc_location(details, device_id):
    P=lx.Project(mode="2D",solver="LSE")
    for i in details:
        P.add_anchor(i[1], i[2])

    t,label=P.add_target()

    for i in details:
        t.add_measure(i[1], i[0])

    P.solve()
    device_queue[device_id]["location"] = t.loc
    return t.loc

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
    distance = rssi_dis.rssi_to_dist(tx_pow, rssi, 14.25)
    
    if device_id not in device_queue:
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" : []}
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
    else:
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_1"]) >= 3:
                location = calc_location(device_queue[device_id]["s_1"], device_id)
                print(location)
                device_queue[device_id]["S_2"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in data base
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_2"]) >= 3:
                location = calc_location(device_queue[device_id]["s_2"], device_id)
                print(location)
                device_queue[device_id]["S_1"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in database
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_3"]) >= 3:
                location = calc_location(device_queue[device_id]["s_3"], device_id)
                print(location)
                device_queue[device_id]["S_1"] = []
                device_queue[device_id]["S_2"] = []

                ## save location in data base


## test the code


import csv

with open('Collected_Data/moving_2m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        print(row[0], row[1], row[2])
        json_data = {"seq_num": int(row[0]), "dev_id": row[2], "tx_pow": 0, "RSSI": int(row[3]), "MAC": row[1]}
        j_d = json.dumps(json_data)
        localization_with_rssi(j_d)
        #time.sleep(0.5)

'''
for seq_num in range(4):
    json_1 = {"seq_num": seq_num, "dev_id": 30, "tx_pow": -20, "RSSI": -37, "MAC": 5007, "acc_x": 0.1, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }
    json_2 = {"seq_num": seq_num, "dev_id": 30, "tx_pow": -20, "RSSI": -37, "MAC": 5008, "acc_x": 0.2, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }
    json_3 = {"seq_num": seq_num, "dev_id": 30, "tx_pow": -20, "RSSI": -37, "MAC": 5009, "acc_x": 0.3, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }

    j_1 = json.dumps(json_1)
    j_2 = json.dumps(json_2)
    j_3 = json.dumps(json_3)

    #print(j_1)

    localization_with_rssi(j_1)
    localization_with_rssi(j_2)
    localization_with_rssi(j_3)


    json_10 = {"seq_num": seq_num, "dev_id": 300, "tx_pow": -20, "RSSI": -43, "MAC": 50007, "acc_x": 0.1, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }
    json_20 = {"seq_num": seq_num, "dev_id": 300, "tx_pow": -20, "RSSI": -43, "MAC": 50008, "acc_x": 0.2, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }
    json_30 = {"seq_num": seq_num, "dev_id": 300, "tx_pow": -20, "RSSI": -43, "MAC": 50009, "acc_x": 0.3, "acc_y": seq_num+2, "acc_z": seq_num+3, "gyro_x": seq_num+1, "gyro_y": seq_num+2, "gyro_z": seq_num+3, "mag_x": seq_num+0.1, "mag_y": seq_num+0.2, "mag_z": seq_num+0.3 }

    j_10 = json.dumps(json_10)
    j_20 = json.dumps(json_20)
    j_30 = json.dumps(json_30)

    #print(j_1)

    localization_with_rssi(j_10)
    localization_with_rssi(j_20)
    localization_with_rssi(j_30)
    #time.sleep(0.2)

print(device_queue)

print(device_queue[30]["location"])
print(device_queue[300]["location"])
'''