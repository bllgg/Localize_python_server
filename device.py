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


class Device:
    G_A = 9.81  # Gravitational acceleration
    pi = math.pi
    dt = 0.1  # Sampling rate = 0.1 s
    Q = 0.5  # defined by experimental values of position prediction with acceleration
    R = 0.5  # defined by experimental values of position prediction with rssi position measuring
    RSSI_CONST = 1.8  # This value is taken by experimental results. Constant regarding to the RSSI distance calculation

    default_position = [0.0, 0.0]  # Entrance of the indoor area. This should be entered by the building authority
    default_variance = [1.8, 1.8]  # Variance of the position by the IMU prediction

    device_name = "default"
    device_data = {}
    # creating the madgwick object for access IMU processing functions.
    sensorfusion = None

    # Disable printings
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    # Restore printings
    def enablePrint(self):
        sys.stdout = sys.__stdout__

    def __init__(self, device_id):
        self.sensorfusion = madgwick.Madgwick(0.5)
        self.device_name = device_id
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
        self.enablePrint(self)

        # Adding calculated location in to the corresponding location of the device queue
        self.device_data["location"] = [t.loc.x, t.loc.y]
        # print (t.loc)
        return [t.loc.x, t.loc.y]


    def imu_earth_ref_accs(self, acc_ary, gyr_ary, mag_ary):
        # this is littlebit tricky. We have to calculate it 10 times. Othervise it will be wrong
        for j in range(10):
            self.sensorfusion.updateRollPitchYaw(acc_ary[0], acc_ary[1], acc_ary[2], gyr_ary[0], gyr_ary[1], gyr_ary[2], mag_ary[0], mag_ary[1], mag_ary[2], self.dt)
        self.sensorfusion.updateRollPitchYaw()
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

        # Position value update according to the kalman gain.
        pos_S = pos_S_bar + K_S * (measured_pos[0] - pos_S_bar)
        pos_E = pos_E_bar + K_E * (measured_pos[1] - pos_E_bar)

        # variance value update according to the kalman gain
        var_S = var_S_bar * (1 - K_S)
        var_E = var_E_bar * (1 - K_E)

        # Update the true speed value with the position difference
        true_speed_S = (pos_S - position[0]) / self.dt
        true_speed_E = (pos_E - position[1]) / self.dt

        # print ('position: [{},{}], speed: [{},{}], variance: [{},{}]'.format(pos_E, pos_N, true_speed_E, true_speed_N, var_E, var_N))
        return [pos_S, pos_E], [true_speed_S, true_speed_E], [var_S, var_E]

    def get_coords_receiver(self, receivers_MAC):
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
        return x, y

    def localization_with_rssi(self, json_data, first_signal):
        # JSON data handling
        json_data = json.loads(json_data)
        device_id = json_data["dev_id"]
        sequence_number = json_data["seq_num"]
        receivers_MAC = json_data["MAC"]
        receiver_x, receiver_y = self.get_coords_receiver(receivers_MAC)  ## collect data from database
        rssi = json_data["RSSI"]
        tx_pow = json_data["tx_pow"]

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
                if len(self.device_queue[device_id]["s_1"]) >= 3:
                    # Do the trilateration
                    if len(self.device_data["s_1"]) == 3:
                        # calculate the location with the RSSI values
                        self.calc_location(self.device_data["s_1"], device_id)

                        # calculate the Earth reference accelration values
                        earth_acc = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)

                        # Filter data with kalman filter
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], earth_acc)

                        print(self.device_data["pos"])

                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_1"], device_id, ) )
                    # thr.start()

                    # Cleaning the other two data buckets
                    self.device_data["s_2"] = []
                    self.device_data["s_3"] = []

                    ##save location in data base
            elif sequence_number % 3 == 1:
                self.device_data["s_2"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
                if len(self.device_data["s_2"]) >= 3:
                    if len(self.device_data["s_2"]) == 3:
                        self.calc_location(self.device_data["s_2"], device_id)
                        earth_acc = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], earth_acc)
                        print(self.device_data["pos"])

                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_2"], device_id, ) )
                    # thr.start()

                    self.device_data["s_1"] = []
                    self.device_data["s_3"] = []

                    ##save location in database
            else:
                self.device_data["s_3"].append([distance, receivers_MAC, (receiver_x, receiver_y)])
                if len(self.device_data["s_3"]) >= 3:
                    if len(self.device_data["s_3"]) == 3:
                        self.calc_location(self.device_data["s_3"], device_id)
                        earth_acc = self.imu_earth_ref_accs(acc_ary, gyr_ary, mag_ary)
                        self.device_data["pos"], self.device_data["speed"], self.device_data["var"] = self.kalman_filter(self.device_data["location"], self.device_data["pos"], self.device_data["speed"], self.device_data["var"], earth_acc)
                        print(self.device_data["pos"])
                    # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                    # thr.start()

                    self.device_data["s_1"] = []
                    self.device_data["s_2"] = []

                    ## save location in data base