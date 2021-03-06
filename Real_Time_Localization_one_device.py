import paho.mqtt.client as mqttClient
import time
import json
import pandas as pd

import rssi_to_distance as rssi_dis
import numpy as np
import localization as lx
from threading import Thread
import math
import sys
import os
import madgwick

G_A = 9.81 # Gravitational acceleration
pi = math.pi
dt = 0.1 # Sampling rate = 0.1 s
Q = 0.5 # defined by experimental values of position prediction with acceleration
R = 0.5 # defined by experimental values of position prediction with rssi position measuring
RSSI_CONST = 1.8 # This value is taken by experimental results. Constant regarding to the RSSI distance calculation

default_position = [1.0, 1.0] # Entrance of the indoor area. This should be entered by the building authority
default_variance = [1.8, 1.8] # Variance of the position by the IMU prediction

# creating the madgwick object for access IMU processing functions.
sensorfusion = madgwick.Madgwick(0.5)

# Disable printings
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore printings
def enablePrint():
    sys.stdout = sys.__stdout__

# This variable is responsible for store online sensor tags and smartphones
device_queue = {}

# Claculate location by the RSSI data
def calc_location(details, device_id):
    blockPrint()
    # Setting up the mode and solver.
    P=lx.Project(mode="2D",solver="LSE")
    # Adding anchor tags of ESP32 devices
    for i in details:
        P.add_anchor(i[1], i[2])
    
    t,label=P.add_target()

    for i in details:
        t.add_measure(i[1], i[0])

    P.solve()
    enablePrint()
    
    # Adding calculated location in to the corresponding location of the device queue
    device_queue[device_id]["location"] = [t.loc.x, t.loc.y]
    # print (t.loc)
    return [t.loc.x, t.loc.y]

def imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary):
    # this is littlebit tricky. We have to calculate it 10 times. Othervise it will be wrong
    for j in range(10):
        sensorfusion.updateRollPitchYaw(acc_ary[0], acc_ary[1], acc_ary[2], gyr_ary[0] + 0.00246, gyr_ary[1] - 0.00185, gyr_ary[2] + 0.00285, mag_ary[0], mag_ary[1], mag_ary[2], dt)

    # Get the quatarnian value of calculated orientation    
    quatarian = sensorfusion.q

    # Find the rotation matrix of the quatarnion
    R = sensorfusion.getRotationMat(quatarian)

    acc_ary = np.array(acc_ary)
    
    # Get the earth reference acceleration values 
    earth_accels = R @ acc_ary

    return earth_accels

# filtering both RSSI and IMU data with Kalman filter
def kalman_filter(measured_pos, position, speed, variance, earth_acc):
    # Speed values update
    speed_S = speed[0] + dt * earth_acc[0]
    speed_E = speed[1] + dt * earth_acc[1]
    # print (earth_acc[0], earth_acc[1])

    ## Prediction phase of the kalman filter
    # position prediction using IMU data
    pos_S_bar = position[0] + dt * speed_S
    pos_E_bar = position[1] + dt * speed_E

    #variance prediction 
    var_S_bar = variance[0] + Q
    var_E_bar = variance[1] + Q

    ## Update phase
    # Kalman gain calculation
    K_S = var_S_bar / (var_S_bar + R)
    K_E = var_E_bar / (var_E_bar + R)

    # Position value update according to the kalman gain.
    pos_S = pos_S_bar + K_S * (measured_pos[0] - pos_S_bar)
    pos_E = pos_E_bar + K_E * (measured_pos[1] - pos_E_bar)

    # variance value update according to the kalman gain
    var_S = var_S_bar * (1 - K_S)
    var_E = var_E_bar * (1 - K_E)

    # Update the true speed value with the position difference
    true_speed_S = (pos_S - position[0]) / dt
    true_speed_E = (pos_E - position[1]) / dt

    # print ('position: [{},{}], speed: [{},{}], variance: [{},{}]'.format(pos_E, pos_N, true_speed_E, true_speed_N, var_E, var_N))
    return [pos_S, pos_E], [true_speed_S, true_speed_E], [var_S, var_E]



'''
temporary function. the coordinates should received from a data base
'''

def get_coords_receiver(receivers_MAC):
    x = 0
    y = 0

    # ESP 1
    if receivers_MAC == "24:6F:28:A9:64:C8":
        x = 1.7
        y = 5.6

    # ESP 2
    if receivers_MAC == "24:6F:28:A9:83:C8":
        x = 8.14
        y = 6.2
    
    # ESP 3
    if receivers_MAC == "24:6F:28:A9:87:40":
        x = 2.8
        y = 0.0
    return x,y

# Core localization algorithm
def localization_with_rssi(json_data):
    # JSON data handling
    json_data = json.loads(json_data)
    device_id = json_data["dev_id"]
    sequence_number = json_data["seq_num"]
    receivers_MAC = json_data["MAC"]
    receiver_x, receiver_y = get_coords_receiver(receivers_MAC) ## collect data from database
    rssi = json_data["RSSI"]
    tx_pow = json_data["tx_pow"]

    # distance calculation with RSSI value
    distance = rssi_dis.rssi_to_dist(tx_pow, rssi, RSSI_CONST)

    acc_ary = [float(json_data["acc_x"]), float(json_data["acc_y"]), float(json_data["acc_z"])]
    gyr_ary = [float(json_data["gyr_x"]), float(json_data["gyr_y"]), float(json_data["gyr_z"])]
    mag_ary = [float(json_data["mag_x"]), float(json_data["mag_y"]), float(json_data["mag_z"])]
    
    # Adding the device into the device queue
    if device_id not in device_queue:
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" :[], "pos" : default_position, "speed" : [0.0, 0.0], "var" : default_variance, }
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
    
    # Calculation the Location 
    else:
        if sequence_number % 3 == 0:
            # Append the data to the device que
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_1"]) >= 3:
                # Do the trilateration 
                if len(device_queue[device_id]["s_1"]) == 3:
                    # calculate the location with the RSSI values
                    calc_location(device_queue[device_id]["s_1"], device_id)

                    # calculate the Earth reference accelration values
                    earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                    
                    # Filter data with kalman filter
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)

                    print(device_queue[device_id]["pos"])

                # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_1"], device_id, ) )
                # thr.start()
                

                # Cleaning the other two data buckets
                device_queue[device_id]["s_2"] = []
                device_queue[device_id]["s_3"] = []
                
                ##save location in data base
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_2"]) >= 3:
                if len(device_queue[device_id]["s_2"]) == 3:
                    calc_location(device_queue[device_id]["s_2"], device_id)
                    earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)
                    print(device_queue[device_id]["pos"])
                
                # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_2"], device_id, ) )
                # thr.start()
                
                device_queue[device_id]["s_1"] = []
                device_queue[device_id]["s_3"] = []
                
                ##save location in database
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_3"]) >= 3:
                if len(device_queue[device_id]["s_3"]) == 3:
                    calc_location(device_queue[device_id]["s_3"], device_id)
                    earth_acc = imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"] = kalman_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], device_queue[device_id]["var"], earth_acc)
                    print(device_queue[device_id]["pos"])
                # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                # thr.start()
                
                device_queue[device_id]["s_1"] = []
                device_queue[device_id]["s_2"] = []

                ## save location in data base



#####################################################################################################

def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    msg = message.payload.decode()
    print (msg)
    localization_with_rssi(msg)
    # value = json.loads(message.payload.decode())
    # # print(type(value))
    # with open('RSSI_exp/8m.csv', 'a', newline='') as f:  # Just use 'w' mode in 3.x
    #     w = csv.writer(f)
    #     w.writerow(value.values())

 
Connected = False   #global variable for the state of the connection

broker_address= "m11.cloudmqtt.com"  #Broker address
port = 17595                         #Broker port
user = "twhkvnkt"
password = "0qLBb25EOa3T"            #Connection password


# broker_address= "192.168.43.125"  #Broker address
# port = 1883                         #Broker port
# user = "mob"
# password = "asdfghjk"            #Connection password
 
client = mqttClient.Client("Python_sub")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop


while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
# client.subscribe("/topic/esp1",2)
# client.subscribe("/topic/esp2",2)
client.subscribe("/topic/esp3",2)
# client.subscribe("/topic/qos1",2)

try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print ("exiting")
    client.disconnect()
    client.loop_stop()

#####################################################################################################
