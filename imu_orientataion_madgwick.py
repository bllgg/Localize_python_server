import time
import madgwick
import csv
import matplotlib.pyplot as plt
import math
from scipy.integrate import cumtrapz
import numpy as np

pi = math.pi
dt = 0.1
sensorfusion = madgwick.Madgwick(0.5)

seq_no=[]
a=0
acc_ary=[]
gyr_ary=[]
mag_ary=[]

roll_ary=[]
pitch_ary=[]
yaw_ary=[]

p_x_ary=[]
p_y_ary=[]
p_z_ary=[]

e_x_ary=[]
e_y_ary=[]
e_z_ary=[]

with open('test.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        seq_no.append(a)
        a+=1
        # acc_ary.append(row[4:7])
        acc_ary = [float(i) for i in row[4:7]]
        # p_x_ary.append(float(row[4]))
        # p_y_ary.append(float(row[5]))
        # p_z_ary.append(float(row[6]))
        p_x_ary.append(acc_ary[0])
        p_y_ary.append(acc_ary[1])
        p_z_ary.append(acc_ary[2])
        # gyr_ary.append(row[7:10])
        # mag_ary.append(row[10:])
        gyr_ary = [float(i) for i in row[7:10]]
        mag_ary = [float(i) for i in row[10:]]
        for j in range(10):
            sensorfusion.updateRollPitchYaw(acc_ary[0], acc_ary[1], acc_ary[2], gyr_ary[0], gyr_ary[1], gyr_ary[2], mag_ary[0], mag_ary[1], mag_ary[2], dt)
        
        roll_ary.append(sensorfusion.roll)
        pitch_ary.append(sensorfusion.pitch)
        yaw_ary.append(sensorfusion.yaw)

        quatarian = sensorfusion.q
        R = sensorfusion.getRotationMat(quatarian)
        # i_R = np.linalg.inv(np.array(R))
        # print(i_R)
        acc_ary = np.array(acc_ary)
        earth_accels = R @ acc_ary

        e_x_ary.append(earth_accels[0])
        e_y_ary.append(earth_accels[1])
        e_z_ary.append(earth_accels[2])



# acc_ary = np.array(acc_ary)
# for i in seq_no:
#     for j in range(10):
#         # newTime = time.time()
#         #newTime - currTime
#         #print(dt)
#         # currTime = newTime

#         sensorfusion.updateRollPitchYaw(float(acc_ary[i][0]), float(acc_ary[i][1]), float(acc_ary[i][2]), float(gyr_ary[i][0]), float(gyr_ary[i][1]), float(gyr_ary[i][2]), float(mag_ary[i][0]), float(mag_ary[i][1]), float(mag_ary[i][2]), dt)
#         # sensorfusion.updateRollPitchYaw(acc_ary[i][0], acc_ary[i][1], acc_ary[i][2], float(gyr_ary[i][0]), float(gyr_ary[i][1]), float(gyr_ary[i][2]), float(mag_ary[i][0]), float(mag_ary[i][1]), float(mag_ary[i][2]), dt)

#     # if print_count == 2:
#     #     # print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))
#     #     print_count = 0

#     roll_ary.append(sensorfusion.roll)
#     pitch_ary.append(sensorfusion.pitch)
#     yaw_ary.append(sensorfusion.yaw)
#     quatarian = sensorfusion.q
#     R = sensorfusion.getRotationMat(quatarian)
#     i_R = np.linalg.inv(np.array(R))
#     print(i_R)
#     earth_accels = i_R @ acc_ary
#     # print_count = print_count + 1
#     # time.sleep(0.01)

# print(roll_ary)

plt.subplot(331)
plt.plot(seq_no, p_x_ary)
plt.ylabel("phone's x_acc")
plt.xlabel("seq num")

plt.subplot(332)
plt.plot(seq_no, p_y_ary)
plt.ylabel("phone's y_acc")
plt.xlabel("seq num")

plt.subplot(333)
plt.plot(seq_no, p_z_ary)
plt.ylabel("phone's z_acc")
plt.xlabel("seq num")

plt.subplot(334)
plt.plot(seq_no, e_x_ary)
plt.ylabel("earth's x_acc")
plt.xlabel("seq num")

plt.subplot(335)
plt.plot(seq_no, e_y_ary)
plt.ylabel("earth's y_acc")
plt.xlabel("seq num")

plt.subplot(336)
plt.plot(seq_no, e_z_ary)
plt.ylabel("earth's z_acc")
plt.xlabel("seq num")

plt.subplot(337)
plt.plot(seq_no, roll_ary)
plt.ylabel("roll angle")
plt.xlabel("seq num")

plt.subplot(338)
plt.plot(seq_no, pitch_ary)
plt.ylabel("pitch angle")
plt.xlabel("seq num")

plt.subplot(339)
plt.plot(seq_no, yaw_ary)
plt.ylabel("yaw angle")
plt.xlabel("seq num")

plt.suptitle("IMU data")

plt.show()