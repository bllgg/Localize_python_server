import csv
import random
import math

# name of csv file
filename = "walk_gen_data.csv"

ESP_1 = "24:6F:28:A9:64:C8"
ESP_2 = "24:6F:28:A9:83:C8"
ESP_3 = "24:6F:28:A9:87:40"

dev_id = 255

tx_pow = -75 # Experimentally found value 
const_n = 1.8 # Experimentally found value

def dist_to_rssi(dist):
    rssi = tx_pow - 10 * const_n * math.log10(dist)
    return rssi

X_coord = [(1 + (5.14/100)*x) for x in range(103)]
Y_coord = [(1 + (2.50/100)*y) for y in range(103)]

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
    RSSI_1 = round(dist_to_rssi(dis_1) + random.randint(0, 1))
    RSSI_2 = round(dist_to_rssi(dis_2) + random.randint(0, 1))
    RSSI_3 = round(dist_to_rssi(dis_3) + random.randint(0, 1))

    row_1.append(str(RSSI_1))
    row_2.append(str(RSSI_2))
    row_3.append(str(RSSI_3))
    

    print(row_2)

    with open(filename, 'a', newline='') as csvfile: 
        # creating a csv writer object  
        csvwriter = csv.writer(csvfile)
        # writing the data rows  
        csvwriter.writerows([row_1])
        csvwriter.writerows([row_2])
        csvwriter.writerows([row_3])

