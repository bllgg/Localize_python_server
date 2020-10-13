import time
import madgwick
import csv
import matplotlib.pyplot as plt
import math
from scipy.integrate import cumtrapz

pi = math.pi

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

with open('mycsvfile.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        seq_no.append(a)
        a+=1
        acc_ary.append(row[4:7])
        p_x_ary.append(float(row[4]))
        p_y_ary.append(float(row[5]))
        p_z_ary.append(float(row[6]))
        gyr_ary.append(row[7:10])
        mag_ary.append(row[10:])


currTime = time.time()
print_count = 0
for i in seq_no:
    for j in range(10):
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.updateRollPitchYaw(float(acc_ary[i][0]), float(acc_ary[i][1]), float(acc_ary[i][2]), float(gyr_ary[i][0]), \
									float(gyr_ary[i][1]), float(gyr_ary[i][2]), float(mag_ary[i][0]), float(mag_ary[i][1]), float(mag_ary[i][2]), dt)


    if print_count == 2:
        # print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))
        print_count = 0

    roll_ary.append(sensorfusion.roll)
    pitch_ary.append(sensorfusion.pitch)
    yaw_ary.append(sensorfusion.yaw)    
    print_count = print_count + 1
    time.sleep(0.01)

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