import trilateration as tr

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

x,y = tr.trackLocation(x1,y1,r1,x2,y2,r2,x3,y3,r3)

print("SensorTag Location:", x, y)

r1 = ((x-x1)**2 + (y-y1)**2)**0.5 +3
r2 = ((x-x2)**2 + (y-y2)**2)**0.5 -5
r3 = ((x-x3)**2 + (y-y3)**2)**0.5 +1

x,y = tr.trackLocation(x1,y1,r1,x2,y2,r2,x3,y3,r3)

print("SensorTag Location:", x, y)

r1 = ((x-x1)**2 + (y-y1)**2)**0.5 -7
r2 = ((x-x2)**2 + (y-y2)**2)**0.5 +5
r3 = ((x-x3)**2 + (y-y3)**2)**0.5

x,y = tr.trackLocation(x1,y1,r1,x2,y2,r2,x3,y3,r3)

print("SensorTag Location:", x, y)

