import rssi_to_distance as rssi_dis
import numpy as np
import localization as lx
from threading import Thread
import math
import json
import time
import sys
import os
import madgwick
import mysql.connector


class Device:
    G_A = 9.81  # Gravitational acceleration
    pi = math.pi
    dt = 0.1  # Sampling rate = 0.1 s
    Q = 0.003  # defined by experimental values of position prediction with acceleration
    R = 2.5  # defined by experimental values of position prediction with rssi position measuring
    RSSI_CONST = 2.3  # This value is taken by experimental results. Constant regarding to the RSSI distance calculation
    tx_power = -67

    default_position = [1.0, 1.0]  # Entrance of the indoor area. This should be entered by the building authority
    default_variance = [0.01, 0.01]  # Variance of the position by the IMU prediction
    temp_position = [1.0, 1.0]
    prev_earth_accelerations = [0, 0, 1]
    earth_accelerations = [0, 0, 1]
    prev_pos_s = 1.0
    prev_pos_e = 1.0
    device_name = "default"
    device_data = {}
    # creating the madgwick object for access IMU processing functions.
    sensorfusion = None
    building_id = None
    esp_devices = {}

    #new variables are needed for moving average.

    # Disable printings
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    # Restore printings
    def enablePrint(self):
        sys.stdout = sys.__stdout__

    # def __init__(self, device_id): 
    def __init__(self, device_id, ils_cursor, ils_db):
        self.sensorfusion = madgwick.Madgwick(0.0)
        self.device_name = device_id
        self.ils_cursor = ils_cursor
        self.ils_db = ils_db
        self.device_data = {"s_1": [], "s_2": [], "s_3": [], "location": [], "pos": self.default_position, "speed": [0.0, 0.0], "var": self.default_variance}

    # Claculate location by the RSSI data
    def calc_location(self, details, device_id):
        self.blockPrint()
        # Setting up the mode and solver.
        P = lx.Project(mode="2D", solver="LSE")
        # Adding anchor tags of ESP32 devices
        for i in details:
            P.add_anchor(i[1], i[2])

        t, label = P.add_target()

        for i in details:
            t.add_measure(i[1], i[0])

        P.solve()
        self.enablePrint()

        # Adding calculated location in to the corresponding location of the device queue
        self.device_data["location"] = [t.loc.x, t.loc.y]
        # print (t.loc)
        return [t.loc.x, t.loc.y]


    def imu_earth_ref_accs(self, acc_ary, gyr_ary, mag_ary):
        # this is littlebit tricky. We have to calculate it 10 times. Othervise it will be wrong
        for j in range(10):
            self.sensorfusion.updateRollPitchYaw(acc_ary[0], acc_ary[1], acc_ary[2], gyr_ary[0], gyr_ary[1], gyr_ary[2], mag_ary[0], mag_ary[1], mag_ary[2], self.dt)
        # self.sensorfusion.updateRollPitchYaw()
        # Get the quatarnian value of calculated orientation
        quatarian = self.sensorfusion.q

        # Find the rotation matrix of the quatarnion
        R = self.sensorfusion.getRotationMat(quatarian)

        acc_ary = np.array(acc_ary)

        # Get the earth reference acceleration values
        earth_accels = R @ acc_ary

        return earth_accels

    # filtering both RSSI and IMU data with Kalman filter
    def kalman_filter(self, measured_pos, position, speed, variance, earth_acc):
        # Speed values update
        speed_S = speed[0] + self.dt * earth_acc[0]
        speed_E = speed[1] + self.dt * earth_acc[1]
        # print (earth_acc[0], earth_acc[1])

        ## Prediction phase of the kalman filter
        # position prediction using IMU data
        pos_S_bar = position[0] + self.dt * speed_S
        pos_E_bar = position[1] + self.dt * speed_E

        # variance prediction
        var_S_bar = variance[0] + self.Q
        var_E_bar = variance[1] + self.Q

        ## Update phase
        # Kalman gain calculation
        K_S = var_S_bar / (var_S_bar + self.R)
        K_E = var_E_bar / (var_E_bar + self.R)
        K_gain = (K_E + K_S) / 2

        # Position value update according to the kalman gain.
        pos_S = pos_S_bar + K_gain * (measured_pos[0] - pos_S_bar)
        pos_E = pos_E_bar + K_gain * (measured_pos[1] - pos_E_bar)
        temp_pos = [pos_S, pos_E]
        pos_S = (pos_S + self.prev_pos_s) / 2
        pos_E = (pos_E + self.prev_pos_e) / 2

        [self.prev_pos_s, self.prev_pos_e] = temp_pos

        # variance value update according to the kalman gain
        var_S = var_S_bar * (1 - K_gain)
        var_E = var_E_bar * (1 - K_gain)

        # Update the true speed value with the position difference
        true_speed_S = (pos_S - position[0]) / self.dt
        true_speed_E = (pos_E - position[1]) / self.dt

        # print ('position: [{},{}], speed: [{},{}], variance: [{},{}]'.format(pos_E, pos_N, true_speed_E, true_speed_N, var_E, var_N))
        return [pos_S, pos_E], [true_speed_S, true_speed_E], [var_S, var_E]

    def get_coords_receiver(self, receivers_MAC):
        x = 0
        y = 0

        if (receivers_MAC not in self.esp_devices):
            # print(receivers_MAC)
            self.ils_cursor.execute("SELECT x_position, y_position, building_id FROM ESP_device where esp_device = \"%s\"" %(receivers_MAC))
            # print('gfd')
            coords = self.ils_cursor.fetchone()
            # print(coords)
            x = coords[0]
            y = coords[1]
            self.building_id = coords[2]
            self.esp_devices[receivers_MAC] = [x, y, self.building_id]
        else:
            x = self.esp_devices[receivers_MAC][0]
            y = self.esp_devices[receivers_MAC][1]

        return x, y

    def localization_with_rssi(self, json_data, first_signal):
        # JSON data handling
        # print('jj')
        json_data = json.loads(json_data)
        device_id = json_data["dev_id"]
        sequence_number = json_data["seq_no"]
        receivers_MAC = json_data["mac"]
        # print('ff')
        receiver_x, receiver_y = self.get_coords_receiver(receivers_MAC)  ## collect data from database
        rssi = json_data["rssi"]
        # print(rssi)
        tx_pow = self.tx_power

        # distance calculation with RSSI value
        distance = rssi_dis.rssi_to_dist(tx_pow, rssi, self.RSSI_CONST)

        acc_ary = [float(json_data["acc_x"]), float(json_data["acc_y"]), float(json_data["acc_z"])]
        gyr_ary = [float(json_data["gyr_x"]), float(json_data["gyr_y"]), float(json_data["gyr_z"])]
        mag_ary = [float(json_data["mag_x"]), float(json_data["mag_y"]), float(json_data["mag_z"])]


        # Adding the device into the device queue
        if first_signal:
            if sequence_number % 3 == 0:
                self.device_data["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            elif sequence_number % 3 == 1:
                self.device_data["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
            else:
                self.device_data["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])

        # Calculation the Location
        else:
            if sequence_number % 3 == 0:
                # Append the data to the device que
                self.device_data["s_1"].append([distance, receivers_MAC, (receiver_x, receiver_y)])

                if len(self.device_data["s_1"]) == 1:
                    # calculate the Earth reference accelration values
                    self.earth_accelerations = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)

                    # averaging the acceleration data
                    temp_acc = self.earth_accelerations
                    self.earth_accelerations = [(self.prev_earth_accelerations[0] + self.earth_accelerations[0]) / 2,
                                                (self.prev_earth_accelerations[1] + self.earth_accelerations[1]) / 2,
                                                (self.prev_earth_accelerations[2] + self.earth_accelerations[2]) / 2]
                    self.prev_earth_accelerations = temp_acc

                    self.device_data["pos"] = self.temp_position

                    # Speed values update
                    speed_S = self.device_data["speed"][0] + self.dt * self.earth_accelerations[0]
                    speed_E = self.device_data["speed"][1] + self.dt * self.earth_accelerations[1]
                    # print (earth_acc[0], earth_acc[1])

                    ## Prediction phase of the kalman filter
                    # position prediction using IMU data
                    pos_S_bar = self.device_data["pos"][0] + self.dt * speed_S
                    pos_E_bar = self.device_data["pos"][1] + self.dt * speed_E

                    self.temp_position = [pos_S_bar, pos_E_bar]

                if len(self.device_data["s_1"]) >= 3:
                    # Do the trilateration
                    if len(self.device_data["s_1"]) == 3:
                        # calculate the location with the RSSI values
                        self.calc_location(self.device_data["s_1"], device_id)

                        # Filter data with kalman filter
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], self.earth_accelerations)
                        self.temp_position = self.device_data["pos"]

                        print(self.device_data["pos"])

                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_1"], device_id, ) )
                    # thr.start()

                    # Cleaning the other two data buckets
                    self.device_data["s_2"] = []
                    self.device_data["s_3"] = []

                    #save location in data base

                    sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s, variance = %s WHERE device_id = %s"
                    # sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s WHERE device_id = %s"
                    val = (str(self.building_id), str(self.device_data["pos"][0]), str(self.device_data["pos"][1]), str(self.device_data["var"][0] + self.device_data["var"][1]), str(device_id))
                    self.ils_cursor.execute(sql, val)

                    self.ils_db.commit()

            elif sequence_number % 3 == 1:
                self.device_data["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])

                if len(self.device_data["s_2"]) == 1:
                    # calculate the Earth reference accelration values
                    self.earth_accelerations = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)

                    # averaging the acceleration data
                    temp_acc = self.earth_accelerations
                    self.earth_accelerations = [(self.prev_earth_accelerations[0] + self.earth_accelerations[0]) / 2,
                                                (self.prev_earth_accelerations[1] + self.earth_accelerations[1]) / 2,
                                                (self.prev_earth_accelerations[2] + self.earth_accelerations[2]) / 2]
                    self.prev_earth_accelerations = temp_acc

                    self.device_data["pos"] = self.temp_position

                    # Speed values update
                    speed_S = self.device_data["speed"][0] + self.dt * self.earth_accelerations[0]
                    speed_E = self.device_data["speed"][1] + self.dt * self.earth_accelerations[1]
                    # print (earth_acc[0], earth_acc[1])

                    ## Prediction phase of the kalman filter
                    # position prediction using IMU data
                    pos_S_bar = self.device_data["pos"][0] + self.dt * speed_S
                    pos_E_bar = self.device_data["pos"][1] + self.dt * speed_E

                    self.temp_position = [pos_S_bar, pos_E_bar]

                if len(self.device_data["s_2"]) >= 3:
                    if len(self.device_data["s_2"]) == 3:
                        self.calc_location(self.device_data["s_2"], device_id)
                        # earth_acc = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], self.earth_accelerations)
                        self.temp_position = self.device_data["pos"]
                        print(self.device_data["pos"])

                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_2"], device_id, ) )
                    # thr.start()

                    self.device_data["s_1"] = []
                    self.device_data["s_3"] = []

                    #save location in database

                    sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s, variance = %s WHERE device_id = %s"
                    # sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s WHERE device_id = %s"
                    val = (str(self.building_id), str(self.device_data["pos"][0]), str(self.device_data["pos"][1]), str(self.device_data["var"][0] + self.device_data["var"][1]), str(device_id))
                    self.ils_cursor.execute(sql, val)

                    self.ils_db.commit()
                    # print(str(self.device_data["var"][0] + self.device_data["var"][1]))
                    # print(self.ils_cursor.rowcount, "record(s) affected")

            else:
                self.device_data["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])

                if len(self.device_data["s_3"]) == 1:
                    # calculate the Earth reference accelration values
                    self.earth_accelerations = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)

                    # averaging the acceleration data
                    temp_acc = self.earth_accelerations
                    self.earth_accelerations = [(self.prev_earth_accelerations[0] + self.earth_accelerations[0]) / 2,
                                                (self.prev_earth_accelerations[1] + self.earth_accelerations[1]) / 2,
                                                (self.prev_earth_accelerations[2] + self.earth_accelerations[2]) / 2]
                    self.prev_earth_accelerations = temp_acc

                    self.device_data["pos"] = self.temp_position

                    # Speed values update
                    speed_S = self.device_data["speed"][0] + self.dt * self.earth_accelerations[0]
                    speed_E = self.device_data["speed"][1] + self.dt * self.earth_accelerations[1]
                    # print (earth_acc[0], earth_acc[1])

                    ## Prediction phase of the kalman filter
                    # position prediction using IMU data
                    pos_S_bar = self.device_data["pos"][0] + self.dt * speed_S
                    pos_E_bar = self.device_data["pos"][1] + self.dt * speed_E

                    self.temp_position = [pos_S_bar, pos_E_bar]

                if len(self.device_data["s_3"]) >= 3:
                    if len(self.device_data["s_3"]) == 3:
                        self.calc_location(self.device_data["s_3"], device_id)
                        # earth_acc = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], self.earth_accelerations)
                        self.temp_position = self.device_data["pos"]
                        print(self.device_data["pos"])
                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                    # thr.start()

                    self.device_data["s_1"] = []
                    self.device_data["s_2"] = []

                    # save location in data 
                    
                    sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s, variance = %s WHERE device_id = %s"
                    # sql = "UPDATE position SET building_id = %s, x_position = %s, y_position = %s WHERE device_id = %s"
                    val = (str(self.building_id), str(self.device_data["pos"][0]), str(self.device_data["pos"][1]), str(self.device_data["var"][0] + self.device_data["var"][1]), str(device_id))
                    self.ils_cursor.execute(sql, val)

                    self.ils_db.commit()

# TEST CODE

# import csv
# ils_db = mysql.connector.connect(
#   host="localhost",
#   user="root",
#   password="",
#   database="Localization"
# )
#
# ils_cursor = ils_db.cursor(buffered=True)
# device_queue = {}
#
# os.system("rm Log_files/Device_walk_gen_data_com_3_res.csv")
#
# with open('gen_data/walk_gen_data_com_3.csv') as csv_file:
#     csv_reader = csv.reader(csv_file, delimiter=',')
#     for row in csv_reader:
#         json_data = {"seq_no": int(row[0]), "dev_id": row[2], "tx_pow": -75, "rssi": int(row[3]), "mac": row[1],
#                      "acc_x": row[4], "acc_y": row[5], "acc_z": row[6], "gyr_x": row[7], "gyr_y": row[8],
#                      "gyr_z": row[9], "mag_x": row[10], "mag_y": row[11], "mag_z": row[12]}
#         msg = json.dumps(json_data)
#         # thr = Thread(target=localization_with_rssi, args=(j_d,))
#         # thr.start()
#         # localization_with_rssi(j_d)
#         device_id = json_data["dev_id"]
#         if device_id not in device_queue:
#             device_queue[device_id] = Device(device_id, ils_cursor, ils_db)
#             device_queue[device_id].localization_with_rssi(msg, True)
#
#         else:
#             device_queue[device_id].localization_with_rssi(msg, False)
