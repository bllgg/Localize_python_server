import rssi_to_distance as rssi_dis
##import trilateration as tr
import localization as lx
import thread
import math
import json

device_queue = {}

def calc_location(details):
    P=lx.Project(mode="2D",solver="LSE")
    for i in details:
        P.add_anchor(i[1], i[2])

    t,label=P.add_target()

    for i in details:
        t.add_measure(i[1], i[0])

    P.solve()

    return t.loc


def localization_with_rssi(json_data):
    json_data = json.load(json_data)
    device_id = json_data["dev_id"]
    sequence_number = json_data["seq_num"]
    receivers_MAC = json_data["MAC"]
    receiver_x = get_x(receivers_MAC) ## collect data from database
    receiver_y = get_y(receivers_MAC) ## collect data from database
    rssi = json_data["RSSI"]
    distance = rssi_dis.rssi_to_dist(0, rssi, 2)
    
    if device_id not in device_queue:
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" = []}
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_0"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
    else:
        if sequence_number % 3 == 0:
            device_queue[device_id]["s_0"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_0"]) >= 3:
                location = calc_location(device_queue[device_id]["s_3"])
                device_queue[device_id]["location"] = location
                ##save location in data base
        elif sequence_number % 3 == 1:
            device_queue[device_id]["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_1"]) >= 3:
                location = calc_location(device_queue[device_id]["s_3"])
                device_queue[device_id]["location"] = location
                ##save location in database
        else:
            device_queue[device_id]["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            if len(device_queue[device_id]["s_3"]) >= 3:
                location = calc_location(device_queue[device_id]["s_3"])
                device_queue[device_id]["location"] = location
                ## save location in data base


## test the code