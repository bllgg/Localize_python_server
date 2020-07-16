import trilateration as tr
import math

x1 = 5
y1 = 40
x2 = 25
y2 = 5
x3 = 45
y3 = 35

x = 25
y = 25
  
  
r1 = ((x-x1)**2 + (y-y1)**2)**0.5 -5
r2 = ((x-x2)**2 + (y-y2)**2)**0.5 -5
r3 = ((x-x3)**2 + (y-y3)**2)**0.5 -5

rs1 = 0 - 10 * 2 * math.log10(r1)
rs2 = 0 - 10 * 2 * math.log10(r2)
rs3 = 0 - 10 * 2 * math.log10(r3)

x,y = tr.trackLocation(x1, y1, x2, y2, x3, y3, rs1, rs2, rs3)

print("SensorTag Location:", x, y)

r1 = ((x-x1)**2 + (y-y1)**2)**0.5 +3
r2 = ((x-x2)**2 + (y-y2)**2)**0.5 -5
r3 = ((x-x3)**2 + (y-y3)**2)**0.5 +1

rs1 = 0 - 10 * 2 * math.log10(r1)
rs2 = 0 - 10 * 2 * math.log10(r2)
rs3 = 0 - 10 * 2 * math.log10(r3)

x,y = tr.trackLocation(x1, y1, x2, y2, x3, y3, rs1, rs2, rs3)

print("SensorTag Location:", x, y)

r1 = ((x-x1)**2 + (y-y1)**2)**0.5 -7
r2 = ((x-x2)**2 + (y-y2)**2)**0.5 +5
r3 = ((x-x3)**2 + (y-y3)**2)**0.5

rs1 = 0 - 10 * 2 * math.log10(r1)
rs2 = 0 - 10 * 2 * math.log10(r2)
rs3 = 0 - 10 * 2 * math.log10(r3)

x,y = tr.trackLocation(x1, y1, x2, y2, x3, y3, rs1, rs2, rs3)

print("SensorTag Location:", x, y)

