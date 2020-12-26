import statistics as st
import csv
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
import math
import numpy as np

seq = [0.001, 1, 2, 3, 4, 5, 6, 7, 8]
means = []
vars = []

with open('0m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('1m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('2m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('3m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('4m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('5m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('6m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('7m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

with open('8m.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    arry = []
    for row in csv_reader:
        arry.append(int(row[3]))
    means.append(st.mean(arry))
    vars.append(st.variance(arry))

print (means)
print (vars)

tx_pow = -67
const_n = 2.3 #Experimentally found value 2.2

def dist_to_rssi(dist):
    rssi = tx_pow - 10 * const_n * math.log10(dist)
    return rssi

rssi_val = []

for x in range(9):
    rssi_val.append(dist_to_rssi(seq[x]))

plt.figure()
seq = np.array(seq)
xnew = np.linspace(seq.min(), seq.max(), 200) 
spl = make_interp_spline(seq, rssi_val, k = 7)
y_smooth = spl(xnew)
plt.plot(xnew, y_smooth, label = "theory")

print(rssi_val)

# plt.figure()
plt.plot(seq, means, label = "Practical")
plt.suptitle("RSSI vals")

plt.legend()

# plt.figure()
# plt.plot(seq, vars)
# plt.suptitle("Vars")

plt.show()
