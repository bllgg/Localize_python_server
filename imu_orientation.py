import numpy as np
import csv
import matplotlib.pyplot as plt
import math
from scipy.integrate import cumtrapz

pi = math.pi
dt = 0.1
def computeOrientation(AccelVals, MagVals):
    roll = np.arctan2(AccelVals[1], AccelVals[2] + 0.05*AccelVals[0])
    pitch = np.arctan2(-1*AccelVals[0], np.sqrt(np.square(AccelVals[1]) + np.square(AccelVals[2])))
    magLength = np.sqrt(np.square(MagVals).sum())
    normMagVals = MagVals/magLength
    yaw = np.arctan2(np.sin(roll)*normMagVals[2] - np.cos(roll)*normMagVals[1],\
                np.cos(pitch)*normMagVals[0] + np.sin(roll)*np.sin(pitch)*normMagVals[1] \
                + np.cos(roll)*np.sin(pitch)*normMagVals[2])

            # roll = np.degrees(roll)
            # pitch = np.degrees(pitch)
            # yaw = np.degrees(yaw)
    return roll, pitch, yaw


# Transform body frame accelerations into the inertial (Earth) frame
    # Set up rotation matrices
def R_x(x):
    # body frame rotation about x axis
    return np.array([[1,      0,       0],
                     [0,np.cos(-x),-np.sin(-x)],
                     [0,np.sin(-x), np.cos(-x)]])
def R_y(y):
    # body frame rotation about y axis
    return np.array([[np.cos(-y),0,-np.sin(-y)],
                    [0,      1,        0],
                    [np.sin(-y), 0, np.cos(-y)]])
def R_z(z):
    # body frame rotation about z axis
    return np.array([[np.cos(-z),-np.sin(-z),0],
                     [np.sin(-z), np.cos(-z),0],
                     [0,      0,       1]])

roll=0
pitch=0
yaw=0

p_x_ary=[]
p_y_ary=[]
p_z_ary=[]

e_x_ary=[]
e_y_ary=[]
e_z_ary=[]

roll_ary=[]
pitch_ary=[]
yaw_ary=[]

seq_no=[]
j=0
with open('mycsvfile.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        seq_no.append(j)
        j+=1
        acc_ary = [float(i) for i in row[4:7]]
        p_x_ary.append(acc_ary[0])
        p_y_ary.append(acc_ary[1])
        p_z_ary.append(acc_ary[2])
        mag_ary = [float(i) for i in row[10:]]
        print(f"phone x acceleration {acc_ary[0]} y acceleration {acc_ary[1]} z acceleration {acc_ary[2]}")
        #print('\n')
        print(f"phone x mag {mag_ary[0]} y mag {mag_ary[1]} z mag {mag_ary[2]}")
        roll, pitch, yaw = computeOrientation(acc_ary, mag_ary)
        roll_ary.append(roll*180/pi)
        pitch_ary.append(pitch*180/pi)
        yaw_ary.append(yaw*180/pi)
        print (f"orientations roll {roll} pitch {pitch} yaw {yaw}")
        acc_ary = np.array(acc_ary)
        earth_accels = R_z(yaw) @ R_y(roll) @ R_x(pitch) @ acc_ary
        e_x_ary.append(earth_accels[0])
        e_y_ary.append(earth_accels[1])
        e_z_ary.append(earth_accels[2])
        print(f'earth x acceleration {earth_accels[0]} y acceleration {earth_accels[1]} z acceleration {earth_accels[2]}')
        print('\n')


# plt.subplot(331)
# plt.plot(seq_no, p_x_ary)
# plt.ylabel("phone's x_acc")
# plt.xlabel("seq num")

# plt.subplot(332)
# plt.plot(seq_no, p_y_ary)
# plt.ylabel("phone's y_acc")
# plt.xlabel("seq num")

# plt.subplot(333)
# plt.plot(seq_no, p_z_ary)
# plt.ylabel("phone's z_acc")
# plt.xlabel("seq num")

# plt.subplot(334)
# plt.plot(seq_no, e_x_ary)
# plt.ylabel("earth's x_acc")
# plt.xlabel("seq num")

# plt.subplot(335)
# plt.plot(seq_no, e_y_ary)
# plt.ylabel("earth's y_acc")
# plt.xlabel("seq num")

# plt.subplot(336)
# plt.plot(seq_no, e_z_ary)
# plt.ylabel("earth's z_acc")
# plt.xlabel("seq num")

# plt.subplot(337)
# plt.plot(seq_no, roll_ary)
# plt.ylabel("roll angle")
# plt.xlabel("seq num")

# plt.subplot(338)
# plt.plot(seq_no, pitch_ary)
# plt.ylabel("pitch angle")
# plt.xlabel("seq num")

# plt.subplot(339)
# plt.plot(seq_no, yaw_ary)
# plt.ylabel("yaw angle")
# plt.xlabel("seq num")

# plt.suptitle("IMU data")

# plt.show()

x_pos =cumtrapz(cumtrapz(e_x_ary,dx=dt),dx=dt)
y_pos =cumtrapz(cumtrapz(e_y_ary,dx=dt),dx=dt)

# plt.subplot(111)
plt.plot(x_pos, y_pos)
plt.suptitle("Position")
plt.show()