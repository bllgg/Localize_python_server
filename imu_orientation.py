import numpy as np

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
                     [0,cos(-x),-sin(-x)],
                     [0,sin(-x), cos(-x)]])
def R_y(y):
    # body frame rotation about y axis
    return np.array([[cos(-y),0,-sin(-y)],
                    [0,      1,        0],
                    [sin(-y), 0, cos(-y)]])
def R_z(z):
    # body frame rotation about z axis
    return np.array([[cos(-z),-sin(-z),0],
                     [sin(-z), cos(-z),0],
                     [0,      0,       1]])

roll=0
pitch=0
yaw=0

roll, pitch, yaw = computeOrientation()

earth_accels = R_z(yaw) @ R_y(roll) @ R_x(pitch) @ accel[:,i]
