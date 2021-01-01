import time
import madgwick
import csv
import matplotlib.pyplot as plt
import math
from scipy.integrate import cumtrapz
import numpy as np
from statistics import mean
from scipy.signal import detrend
from statistics import variance
from statistics import stdev
import rssi_to_distance as rsd

G_A = 9.81

pi = math.pi
dt = 0.1
sensorfusion = madgwick.Madgwick(0.0)

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

p_x_gyr=[]
p_y_gyr=[]
p_z_gyr=[]

e_x_ary=[]
e_y_ary=[]
e_z_ary=[]

prev_ar = [0, 0, 0]
new_arr = [0, 0, 0]
mean_eax = 0
mean_eay = 0

distance_vals = []
rssi_vals = []
mag_x_vals = []
mag_y_vals = []
mag_z_vals = []

with open('gen_data/stat_gen_data_com_1.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        seq_no.append(a)
        a+=1
        # acc_ary.append(row[4:7])
        rssi_vals.append(int(row[3]))
        distance_vals.append(rsd.rssi_to_dist(-75, int(row[3]), 1.8))
        mag_x_vals.append(int(float(row[10])))
        mag_y_vals.append(int(float(row[11])))
        mag_z_vals.append(int(float(row[12])))

        acc_ary = [round(float(i),3)*G_A for i in row[4:7]]
        new_arr = [a + b for a, b in zip(acc_ary, prev_ar)]
        prev_ar = [number / 2 for number in new_arr]

        acc_ary = prev_ar
        # p_x_ary.append(float(row[4]))
        # p_y_ary.append(float(row[5]))
        # p_z_ary.append(float(row[6]))
        p_x_ary.append(acc_ary[0])
        p_y_ary.append(acc_ary[1])
        p_z_ary.append(acc_ary[2])
        # gyr_ary.append(row[7:10])
        # mag_ary.append(row[10:])
        gyr_ary = [round(float(i),3) for i in row[7:10]]
        
        p_x_gyr.append(gyr_ary[0])
        p_y_gyr.append(gyr_ary[1])
        p_z_gyr.append(gyr_ary[2])

        mag_ary = [round(float(i),3) for i in row[10:]]
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

        e_x_ary.append(round(earth_accels[0], 2))
        e_y_ary.append(round(earth_accels[1], 2))
        e_z_ary.append(round(earth_accels[2], 2))


plt.figure()
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
plt.ylabel("earth's South")
plt.xlabel("seq num")

plt.subplot(335)
plt.plot(seq_no, e_y_ary)
plt.ylabel("earth's East")
plt.xlabel("seq num")

plt.subplot(336)
plt.plot(seq_no, e_z_ary)
plt.ylabel("earth's Down")
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

# plt.show()
plt.figure()
x_speed = cumtrapz(e_x_ary,dx=0.1)
y_speed = cumtrapz(e_y_ary,dx=0.1)

plt.subplot(121)
plt.hist(x_speed, bins = 100)
plt.ylabel("frequency")
plt.xlabel("speed South")
plt.subplot(122)
plt.hist(y_speed, bins = 100)
plt.ylabel("frequency")
plt.xlabel("speed East")
plt.suptitle("Earth reference speed histogram")

plt.figure()
x_pos = cumtrapz(cumtrapz(e_x_ary,dx=0.1),dx=0.1)
y_pos = cumtrapz(cumtrapz(e_y_ary,dx=0.1),dx=0.1)

plt.plot(seq_no[2:], y_pos)
plt.suptitle("East Position")

plt.figure()
plt.plot(seq_no[2:], x_pos)
plt.suptitle("South Position")

plt.figure()
x_v = cumtrapz(e_x_ary,dx=0.1)
y_v = cumtrapz(e_y_ary,dx=0.1)

plt.plot(seq_no[1:], y_v)
plt.suptitle("East Speed")

plt.figure()
plt.plot(seq_no[1:], x_v)
plt.suptitle("South Speed")
# plt.subplot(111)

print("mean s x", mean(p_x_ary))
print("stddev s x", stdev(p_x_ary))
print("mean s y", mean(p_y_ary))
print("stddev s y", stdev(p_y_ary))
print("mean s z", mean(p_z_ary))
print("stddev s z", stdev(p_z_ary))

print("mean south", mean(e_x_ary))
print("mean east", mean(e_y_ary))
print("mean down", mean(e_z_ary))
print("variance south", variance(e_x_ary))
print("variance east", variance(e_y_ary))
print("variance down", variance(e_z_ary))

print("mean x gyr", mean(p_x_gyr))
print("stddev x gyr", stdev(p_x_gyr))
print("mean y gyr", mean(p_y_gyr))
print("stddev y gyr", stdev(p_y_gyr))
print("mean z gyr", mean(p_z_gyr))
print("stddev z gyr", stdev(p_z_gyr))

# print(rssi_vals)
print("mean of RSSI", mean(rssi_vals))
print("variance of RSSI", variance(rssi_vals))
print("mean of Distances", mean(distance_vals))
print("variance of Distances", variance(distance_vals))

print("stddev x mag", stdev(mag_x_vals))
print("stddev y mag", stdev(mag_y_vals))
print("stddev z mag", stdev(mag_z_vals))

plt.figure()
plt.subplot(121)
plt.hist(e_x_ary, bins = 100)
plt.ylabel("frequency")
plt.xlabel("e_r_acceleration South")
plt.subplot(122)
plt.hist(e_y_ary, bins = 100)
plt.ylabel("frequency")
plt.xlabel("e_r_acceleration East")
plt.suptitle("Earth Reference acceleration histogram")

plt.show()


