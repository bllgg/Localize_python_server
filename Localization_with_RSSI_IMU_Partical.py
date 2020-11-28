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

# need for partical filtering
import scipy as scipy
from numpy.random import uniform
import scipy.stats
import random

np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)
import cv2


G_A = 9.81 # Gravitational acceleration
pi = math.pi
dt = 0.1 # Sampling rate = 0.1 s
Q = 0.5 # defined by experimental values of position prediction with acceleration
R = 0.5 # defined by experimental values of position prediction with rssi position measuring
RSSI_CONST = 1.8 # This value is taken by experimental results. Constant regarding to the RSSI distance calculation

default_position = [1.0, 1.0] # Entrance of the indoor area. This should be entered by the building authority
# default_variance = [1.8, 1.8] # Variance of the position by the IMU prediction


# Constants and varibles for partical filtering
WIDTH = 815
HEIGHT = 781
WINDOW_NAME = "Indoor Map"

# sensor_std_err=5 # this is similar to Q or R but for random number generation there for no need of using this variable
# x_range = np.array([0,WIDTH/100.0])
# y_range = np.array([0,HEIGHT/100.0])
x_range = np.array([0,2.0])
y_range = np.array([0,2.0])

#Number of partciles
N = 40

def create_uniform_particles(x_range, y_range, N):
    particles = np.empty((N, 2))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    return particles

#landmarks=np.array([ [144,73], [410,13], [336,175], [718,159], [178,484], [665,464] ])
landmarks = np.array([ [1.7, 5.6], [8.14, 6.20], [2.8, 0]])
walls = np.array([[[0, 780 - 670], [364, 780 - 670]], [[0, 780 - 560], [364, 780 - 560]], [[0, 780 - 0], [0, 780 - 670]], [[0, 780 - 0], [454, 780 - 0]], [[454, 780 - 0], [454, 780 - 130]], [[454, 780 - 130], [814, 780 - 130]], [[814, 780 - 130], [814, 780 - 780]], [[814, 780 - 780],[364, 780 - 780]]])
NL = len(landmarks)
particles = create_uniform_particles(x_range, y_range, N)

weights = np.array([1.0]*N)

# Create a black image, a window and bind the function to window
img = np.zeros((HEIGHT,WIDTH,3), np.uint8)
cv2.namedWindow(WINDOW_NAME)
# cv2.setMouseCallback(WINDOW_NAME,mouseCallback)

center = np.array([[-10,-10]])

# position = np.array([[0, 0]])
trajectory = np.zeros(shape=(0,2))
robot_pos = np.zeros(shape=(0,2))
DELAY_MSEC = 50

# creating the madgwick object for access IMU processing functions.
sensorfusion = madgwick.Madgwick(0.5)

# functions need for visualize results
def drawLines(img, points, r, g, b):
    cv2.polylines(img, [np.int32(points)], isClosed=False, color=(r, g, b))

def drawCross(img, center, r, g, b):
    d = 5
    t = 2
    LINE_AA = cv2.LINE_AA if cv2.__version__[0] >= '3' else cv2.CV_AA
    color = (r, g, b)
    ctrx = center[0,0]
    ctry = center[0,1]
    cv2.line(img, (ctrx - d, ctry - d), (ctrx + d, ctry + d), color, t, LINE_AA)
    cv2.line(img, (ctrx + d, ctry - d), (ctrx - d, ctry + d), color, t, LINE_AA)

# Prediction stage is over and complete
def predict(particles, u):
    particles[:, 0] += u[0]
    particles[:, 1] += u[1]
    return particles

def update(particles, weights, z, R, landmarks):
    weights.fill(1.)
    for i, landmark in enumerate(landmarks):
       
        distance = np.power((particles[:,0] - landmark[0])**2 +(particles[:,1] - landmark[1])**2,0.5)
        weights *= scipy.stats.norm(distance, R).pdf(z[i])

 
    weights += 1.e-300 # avoid round-off to zero
    weights /= sum(weights)

def systematic_resample(weights):
    N = len(weights)
    positions = (np.arange(N) + np.random.random()) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N and j<N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights)

# Finished partical filter
# filtering both RSSI and IMU data with partical filter
def partical_filter(measured_pos, position, speed, earth_acc):
    # global position
    global trajectory
    # global previous_x
    # global previous_y
    global zs
    
    print("Before calculate", position)
    # center=np.array([[x,y]])
    measured_position = np.array(measured_pos)
    print("Measured Position", measured_position)
    # trajectory = np.vstack((trajectory,np.array([int(position[0] * 100), 780 - int(position[0] * 100)]))) # should go tho the last
    # trajectory = np.vstack((trajectory,np.array([int(measured_pos[0] * 100), 780 - int(measured_pos[0] * 100)]))) # should go tho the last

    # Speed values update
    print("before Speed",speed[0], speed[1])
    speed_S = speed[0] + dt * earth_acc[0]
    speed_E = speed[1] + dt * earth_acc[1]
    print("Accelerations", earth_acc[0], earth_acc[1])
    print("after Speed",speed_S, speed_E)

    ## Prediction phase of the Particle filter
    # position prediction using IMU data

    S_distance = dt * speed_S
    E_distance = dt * speed_E

    u = np.array([S_distance, E_distance])
    print("U value",u)
    
    ############################################################################
    pos_S = sum(particles[:,0]) / N
    pos_E = sum(particles[:,1]) / N
    print("positions from particals", pos_S, pos_E)

    true_speed_S = (pos_S - position[0]) / dt
    true_speed_E = (pos_E - position[1]) / dt
    ############################################################################

    # Move
    predict(particles, u)

    zs = (np.linalg.norm(landmarks - measured_position, axis=1))
    print("Zs value", zs)
    update(particles, weights, z=zs, R=5, landmarks=landmarks)
    
    indexes = systematic_resample(weights)
    resample_from_index(particles, weights, indexes)

    # pos_S = sum(particles[:,0]) / N
    # pos_E = sum(particles[:,1]) / N
    # print("positions from particals", pos_S, pos_E)

    # true_speed_S = (pos_S - position[0]) / dt
    # true_speed_E = (pos_E - position[1]) / dt

    print("True speed values", true_speed_S, true_speed_E)

    position = np.array([pos_S, pos_E])
    print("After calculation", position)
    print(" ")
    # true_speed_S = (measured_pos[0] - measured_position[0]) / dt
    # true_speed_E = (measured_pos[1] - measured_position[1]) / dt

    return [measured_pos[0], measured_pos[0]], [true_speed_S, true_speed_E]

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
    global img
    ext_pos = []
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
        device_queue[device_id] = {"s_1":[], "s_2":[], "s_3":[], "location" :[], "pos" : default_position, "speed" : [0.0, 0.0]}
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
                    
                    # print(device_queue[device_id]["speed"])
                    # Filter data with kalman filter
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"] = partical_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], earth_acc)

                    # print(device_queue[device_id]["speed"])
                    # ext_pos = device_queue[device_id]["pos"]

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
                    # print(device_queue[device_id]["speed"])
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"] = partical_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], earth_acc)
                    # print(device_queue[device_id]["speed"])
                    # ext_pos = device_queue[device_id]["pos"]
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
                    # print(device_queue[device_id]["speed"])
                    device_queue[device_id]["pos"], device_queue[device_id]["speed"] = partical_filter(device_queue[device_id]["location"], device_queue[device_id]["pos"], device_queue[device_id]["speed"], earth_acc)
                    # print(device_queue[device_id]["speed"])
                    # ext_pos = device_queue[device_id]["pos"]
                # thr = Thread(target=calc_location, args=(device_queue[device_id]["s_3"], device_id, ) )
                # thr.start()
                
                device_queue[device_id]["s_1"] = []
                device_queue[device_id]["s_2"] = []

                ## save location in data base
    
    # time.sleep(0.1)
    # return ext_pos

## test the code

import csv

print("Without threading")

start_time = time.time()
with open('test_3.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        imS = cv2.resize(img, (651, 624)) 
        cv2.imshow(WINDOW_NAME,imS)
        img = np.zeros((HEIGHT,WIDTH,3), np.uint8)
        # if (j == 2):
        #     cv2.waitKey(0)
        # drawing perimeters
        for wall in walls:
            drawLines(img, wall, 255, 0, 255)
    
        # #landmarks
        for landmark in landmarks:
            cv2.circle(img,tuple([int(landmark[0]*100), 780 - int(landmark[1]*100)]),20,(255,0,0),-1)

        # drawLines(img, trajectory,   0,   255, 0)
        # drawCross(img, center, r=255, g=0, b=0)
        
        # draw_particles:
        for particle in particles:
            cv2.circle(img,tuple((int(particle[0]),int(particle[1]))),1,(255,255,255),-1)

        if cv2.waitKey(DELAY_MSEC) & 0xFF == 27:
            break

        # cv2.waitKey(0)
        json_data = {"seq_num": int(row[0]), "dev_id": row[2], "tx_pow": -75, "RSSI": int(row[3]), "MAC": row[1], "acc_x": row[4], "acc_y": row[5], "acc_z": row[6], "gyr_x": row[7], "gyr_y": row[8], "gyr_z": row[9], "mag_x": row[10], "mag_y": row[11], "mag_z": row[12]}
        j_d = json.dumps(json_data)
        #thr = Thread(target=localization_with_rssi, args=(j_d,))
        #thr.start()
        localization_with_rssi(j_d)
        # print(device_queue[row[2]]["pos"])

        time.sleep(0.01)


stop_time = time.time()

print("--- %s seconds ---" % (time.time() - start_time))

cv2.waitKey(0)
cv2.destroyAllWindows()