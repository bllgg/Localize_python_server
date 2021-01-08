import numpy as num
from localization import geometry as gx
from scipy.optimize import minimize
from localization.geometry import *

class Anchor:
    def __init__(self, ID, loc):
        self.loc = loc
        self.ID = str(ID)

    def __str__(self):
        return 'Anchor ' + self.ID + ' @ ' + self.loc.__str__()


class Target:
    def __init__(self, ID):
        self.loc = None
        self.ID = str(ID)
        self.measures = []

    def __str__(self):
        if self.loc is None:
            return 'Target ' + self.ID
        else:
            return 'Target ' + self.ID + ' @ Real Location:' + self.loc.__str__()

    def add_measure(self, a, d):
        self.measures.append((a, d))

class Project:
    def __init__(self):
        self.AnchorDic = {}
        self.TargetDic = {}
        self.nt = 0

    def add_anchor(self, ID, loc):
        try:
            self.AnchorDic[ID]
            print(str(ID) + ':Anchor with same ID already exists')
            return
        except KeyError:
            a = Anchor(ID, point(loc))
            self.AnchorDic[ID] = a
        return a

    def add_target(self, ID=None):
        try:
            self.TargetDic[ID]
            print('Target with same ID already exists')
            return
        except:
            self.nt = self.nt + 1
            if ID:
                pass
            else:
                ID = 't' + str(self.nt)
            t = Target(ID)
            self.TargetDic[ID] = t
        return (t, ID)

    def solve(self, **kwargs):
        for tID in self.TargetDic.keys():
            tar = self.TargetDic[tID]
            cA = []
            for tup in tar.measures:
                landmark = tup[0]
                c = self.AnchorDic[landmark].loc
                d = tup[1]
                cA.append(circle(c, d))
            tar.loc = lse(cA)

def Norm(x, y):
    return ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2) ** .5


def sum_error(x, c, r):
    l = len(c)
    e = 0
    for i in range(l):
        e = e + (Norm(x, c[i].std()) - r[i]) ** 2
    return e


def lse(cA):
    l = len(cA)
    r = [w.r for w in cA]
    c = [w.c for w in cA]
    S = sum(r)
    W = [(S - w) / ((l - 1) * S) for w in r]
    p0 = gx.point(0, 0, 0)  # Initialized point
    for i in range(l):
        p0 = p0 + W[i] * c[i]
    x0 = num.array([p0.x, p0.y])
    print (x0[1])
    print('LSE Geolocating...')
    res = minimize(sum_error, x0, args=(c, r), method='BFGS')
    ans = res.x
    return gx.point(ans)

esp = [[0,0],[2,0],[1,1.73]]
tag = [1.54, 1.54, 1.54]
# Setting up the mode and solver.
P = Project()

count = 0

# Adding anchor tags of ESP32 devices
for i in esp:
    count = count+1
    P.add_anchor(count, i)

t, label = P.add_target()

count = 0

for i in tag:
    count = count+1
    t.add_measure(count, i)

P.solve()
print (t.loc)