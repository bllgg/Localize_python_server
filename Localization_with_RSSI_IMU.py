import rssi_to_distance as rssi_dis
import numpy as np
##import trilateration as tr
import localization as lx
from threading import Thread
import math
import json
import time
import sys
import os
import madgwick

G_A = 9.81
pi = math.pi
dt = 0.1
Q = 0.5 # defined by experimental values of position prediction with acceleration
R = 0.5 # defined by experimental values of position prediction with rssi position measuring
sensorfusion = madgwick.Madgwick(0.5)

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
    device_queue[device_id]["location"] = [t.loc.x, t.loc.y]
    print (t.loc)
    return [t.loc.x, t.loc.y]

def imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary):
    for j in range(10):
        sensorfusion.updateRollPitchYaw(acc_ary[0], acc_ary[1], acc_ary[2], gyr_ary[0] + 0.00246, gyr_ary[1] - 0.00185, gyr_ary[2] + 0.00285, mag_ary[0], mag_ary[1], mag_ary[2], dt)
    
    quatarian = sensorfusion.q
    R = sensorfusion.getRotationMat(quatarian)
    acc_ary = np.array(acc_ary)
    earth_accels = R @ acc_ary

    return earth_accels

def kalman_filter(measured_pos, position, speed, variance, earth_acc):
    speed_x = speed[0] + dt * earth_acc[0]
    speed_y = speed[1] + dt * earth_acc[1]

    # Prediction phase
    pos_x_bar = position[0] + dt * speed_x
    pos_y_bar = position[1] + dt * speed_y

    var_x_bar = variance[0] + Q
    var_y_bar = variance[1] + Q

    # Update phase
    K_x = var_x_bar / (var_x_bar + R)
    K_y = var_y_bar / (var_y_bar + R)

    pos_x = pos_x_bar + K_x * (measured_pos[0] - pos_x_bar)
    pos_y = pos_y_bar + K_y * (measured_pos[1] - pos_y_bar)

    var_x = var_x_bar * (1 - K_x)
    var_y = var_y_bar * (1 - K_y)

    true_speed_x = (pos_x - position[0]) / dt
    true_speed_y = (pos_y - position[1]) / dt

    print ('position: [{},{}], speed: [{},{}], variance: [{},{}]'.format(pos_x, pos_y, true_speed_x, true_speed_y, var_x, var_y))
    return [pos_x, pos_y], [true_speed_x, true_speed_y], [var_x, var_y]



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
    acc_ary = [float(json_data["acc_x"]), float(json_data["acc_y"]), float(json_data["acc_z"])]
    gyr_ary = [float(json_data["gyr_x"]), float(json_data["gyr_y"]), float(json_data["gyr_z"])]
    mag_ary = [float(json_data["mag_x"]), float(json_data["mag_y"]), float(json_data["mag_z"])]
    # print(type(acc_ary[0]))
    earth_acc = []

    if device_id not in device_queue:
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" :[], "pos" : [0.0, 0.0], "speed" : [0.0, 0.0], "var" : [0.0, 0.0], }
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
                earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_1"], device_id, ) )
                #thr.start()
                print("1")

                device_queue[device_id]["S_2"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in data base
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_2"]) == 3:
                calc_location(device_queue[device_id]["s_2"], device_id)
                earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_2"], device_id, ) )
                #thr.start()
                print("2")
                device_queue[device_id]["S_1"] = []
                device_queue[device_id]["S_3"] = []
                
                ##save location in database
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_3"]) == 3:
                calc_location(device_queue[device_id]["s_3"], device_id)
                earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)
                #thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                #thr.start()
                print("3")
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
        json_data = {"seq_num": int(row[0]), "dev_id": row[2], "tx_pow": -24, "RSSI": int(row[3]), "MAC": row[1], "acc_x": row[4], "acc_y": row[5], "acc_z": row[6], "gyr_x": row[7], "gyr_y": row[8], "gyr_z": row[9], "mag_x": row[10], "mag_y": row[11], "mag_z": row[12]}
        j_d = json.dumps(json_data)
        #thr = Thread(target=localization_with_rssi, args=(j_d,))
        #thr.start()
        localization_with_rssi(j_d)
stop_time = time.time()

# enablePrint()
print("--- %s seconds ---" % (time.time() - start_time))
