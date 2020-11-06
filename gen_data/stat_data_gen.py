import csv
import random
import math

from numpy.random import seed
from numpy.random import randn

seed(1)

# name of csv file
filename_3 = "stat_gen_data_com_3.csv"
filename_1 = "stat_gen_data_com_1.csv"

ESP_1 = "24:6F:28:A9:64:C8"
ESP_2 = "24:6F:28:A9:83:C8"
ESP_3 = "24:6F:28:A9:87:40"

dev_id = 255

tx_pow = -75 # Experimentally found value 
const_n = 1.8 # Experimentally found value

def dist_to_rssi(dist):
    rssi = tx_pow - 10 * const_n * math.log10(dist)
    return rssi

X_coord = [1.0] * 103
Y_coord = [1.0] * 103

for itr in range(103):
    row_1 = []
    row_2 = []
    row_3 = []
    
    row_1.append(str(itr))
    row_2.append(str(itr))
    row_3.append(str(itr))

    row_1.append(ESP_1)
    row_2.append(ESP_2)
    row_3.append(ESP_3)

    row_1.append(str(dev_id))
    row_2.append(str(dev_id))
    row_3.append(str(dev_id))

    # Calculating distances
    dis_1 = ((X_coord[itr] - 1.70)**2 + (Y_coord[itr] - 5.6)**2)**0.5
    dis_2 = ((X_coord[itr] - 8.14)**2 + (Y_coord[itr] - 6.2)**2)**0.5
    dis_3 = ((X_coord[itr] - 2.80)**2 + (Y_coord[itr] - 0.0)**2)**0.5

    # print(dis_1, dis_2, dis_3)

    # Calculating RSSI values
    rssi_vals = randn(3)
    RSSI_1 = int(round(dist_to_rssi(dis_1) + 2.5 * rssi_vals[0]))
    RSSI_2 = int(round(dist_to_rssi(dis_2) + 2.5 * rssi_vals[1]))
    RSSI_3 = int(round(dist_to_rssi(dis_3) + 2.5 * rssi_vals[2]))

    row_1.append(str(RSSI_1))
    row_2.append(str(RSSI_2))
    row_3.append(str(RSSI_3))

    # Calculating Acceleration values
    vals_acc = randn(3)
    acc_x = round(0.0 + vals_acc[0] * 0.003, 6)
    acc_y = round(0.0 + vals_acc[1] * 0.003, 6)
    acc_z = round(0.98 + vals_acc[2] * 0.003, 6)

    row_1.append(str(acc_x))
    row_1.append(str(acc_y))
    row_1.append(str(acc_z))

    vals_acc = randn(3)
    acc_x = round(0.0 + vals_acc[0] * 0.003, 6)
    acc_y = round(0.0 + vals_acc[1] * 0.003, 6)
    acc_z = round(0.98 + vals_acc[2] * 0.003, 6)

    row_2.append(str(acc_x))
    row_2.append(str(acc_y))
    row_2.append(str(acc_z))

    vals_acc = randn(3)
    acc_x = round(0.0 + vals_acc[0] * 0.003, 6)
    acc_y = round(0.0 + vals_acc[1] * 0.003, 6)
    acc_z = round(0.98 + vals_acc[2] * 0.003, 6)

    row_3.append(str(acc_x))
    row_3.append(str(acc_y))
    row_3.append(str(acc_z))
    
    # Calculating gyro values
    vals_gyr = randn(3)
    gyr_x = round(0.0 + vals_gyr[0] * 0.01, 6)
    gyr_y = round(0.0 + vals_gyr[1] * 0.01, 6)
    gyr_z = round(0.0 + vals_gyr[2] * 0.01, 6)

    row_1.append(str(gyr_x))
    row_1.append(str(gyr_y))
    row_1.append(str(gyr_z))

    vals_gyr = randn(3)
    gyr_x = round(0.0 + vals_gyr[0] * 0.01, 6)
    gyr_y = round(0.0 + vals_gyr[1] * 0.01, 6)
    gyr_z = round(0.0 + vals_gyr[2] * 0.01, 6)

    row_2.append(str(gyr_x))
    row_2.append(str(gyr_y))
    row_2.append(str(gyr_z))

    vals_gyr = randn(3)
    gyr_x = round(0.0 + vals_gyr[0] * 0.01, 6)
    gyr_y = round(0.0 + vals_gyr[1] * 0.01, 6)
    gyr_z = round(0.0 + vals_gyr[2] * 0.01, 6)

    row_3.append(str(gyr_x))
    row_3.append(str(gyr_y))
    row_3.append(str(gyr_z))

    # Calculating Mag values
    vals_mag = randn(3)
    mag_x = round(41.0 + vals_mag[0] * 2.1, 1)
    mag_y = round(0.0 + vals_mag[1] * 2.1, 1)
    mag_z = round(0.0 + vals_mag[2] * 2.1, 1)

    row_1.append(str(mag_x))
    row_1.append(str(mag_y))
    row_1.append(str(mag_z))

    vals_mag = randn(3)
    mag_x = round(41.0 + vals_mag[0] * 2.1, 1)
    mag_y = round(0.0 + vals_mag[1] * 2.1, 1)
    mag_z = round(0.0 + vals_mag[2] * 2.1, 1)

    row_2.append(str(mag_x))
    row_2.append(str(mag_y))
    row_2.append(str(mag_z))

    vals_mag = randn(3)
    mag_x = round(41.0 + vals_mag[0] * 2.1, 1)
    mag_y = round(0.0 + vals_mag[1] * 2.1, 1)
    mag_z = round(0.0 + vals_mag[2] * 2.1, 1)

    row_3.append(str(mag_x))
    row_3.append(str(mag_y))
    row_3.append(str(mag_z))

    # print(row_2)

    with open(filename_3, 'a', newline='') as csvfile: 
        # creating a csv writer object  
        csvwriter = csv.writer(csvfile)
        # writing the data rows  
        csvwriter.writerows([row_1])
        csvwriter.writerows([row_2])
        csvwriter.writerows([row_3])

    with open(filename_1, 'a', newline='') as csvfile: 
        # creating a csv writer object  
        csvwriter = csv.writer(csvfile)
        # writing the data rows  
        csvwriter.writerows([row_1])