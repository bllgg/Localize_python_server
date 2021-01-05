import numpy as np
import time
import csv
import cv2
import math

np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)


def drawLines(img, points, r, g, b):
    cv2.polylines(img, [np.int32(points)], isClosed=False, color=(r, g, b))


def drawCross(img, center, r, g, b):
    d = 5
    t = 2
    LINE_AA = cv2.LINE_AA if cv2.__version__[0] >= '3' else cv2.CV_AA
    color = (r, g, b)
    ctrx = center[0, 0]
    ctry = center[0, 1]
    cv2.line(img, (ctrx - d, ctry - d), (ctrx + d, ctry + d), color, t, LINE_AA)
    cv2.line(img, (ctrx + d, ctry - d), (ctrx - d, ctry + d), color, t, LINE_AA)


WIDTH = 815
HEIGHT = 781
WINDOW_NAME="Data visualizer"

x_range = np.array([0, WIDTH])
y_range = np.array([0, HEIGHT])

# landmarks=np.array([ [144,73], [410,13], [336,175], [718,159], [178,484], [665,464] ])
landmarks = np.array([[170, 780 - 560], [814, 780 - 620], [280, 780 - 0]])
walls = np.array(
    [[[0, 780 - 670], [364, 780 - 670]], [[0, 780 - 560], [364, 780 - 560]], [[0, 780 - 0], [0, 780 - 670]],
     [[0, 780 - 0], [454, 780 - 0]], [[454, 780 - 0], [454, 780 - 130]], [[454, 780 - 130], [814, 780 - 130]],
     [[814, 780 - 130], [814, 780 - 780]], [[814, 780 - 780], [364, 780 - 780]]])
NL = len(landmarks)

# Create a black image, a window and bind the function to window
img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
cv2.namedWindow(WINDOW_NAME)

center = np.array([[-10, -10]])

trajectory = np.zeros(shape=(0, 2))
robot_pos = np.zeros(shape=(0, 2))
previous_x = -1
previous_y = -1
DELAY_MSEC = 50


# with open('stat_only_from_rssi.csv') as csv_file:  # only RSSI stationary test
# with open('walk_only_from_rssi.csv') as csv_file:  # only RSSI walking test
with open('stat_from_kalman_filter_after_tune_2.csv') as csv_file:  # kalman walking test before tune
# with open('stat_from_kalman_filter_before_tune.csv') as csv_file:  # kalman stationary test before tune
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if cv2.waitKey(DELAY_MSEC) & 0xFF == 27:
            break
        imS = cv2.resize(img, (651, 624))
        cv2.imshow(WINDOW_NAME, imS)
        img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

        # walls
        for wall in walls:
            drawLines(img, wall, 255, 0, 255)

        # landmarks (ESP 32 s)
        for landmark in landmarks:
            cv2.circle(img, tuple(landmark), 20, (255, 0, 0), -1)

        # r_pos_x = (float(row[0]) * math.cos(np.radians(-17)) - float(row[1]) * math.sin(np.radians(-17)))*100
        # r_pos_y = 781 - (float(row[0]) * math.sin(np.radians(-17)) + float(row[1]) * math.cos(np.radians(-17)))*100
        r_pos_x = float(row[0]) * 100
        r_pos_y = 781 - float(row[1]) * 100
        real_pos = np.array([[int(r_pos_x), int(r_pos_y)]])
        trajectory = np.vstack((trajectory, real_pos))
        drawLines(img, trajectory, 0, 255, 0)
        drawCross(img, center, r=255, g=0, b=0)
        drawCross(img, real_pos, r=255, g=0, b=255)
        drawCross(img, np.array([[200, 780 - 200]]), b=255, g=0, r=0)  # the physical position
        time.sleep(0.1)

cv2.waitKey(0)
cv2.destroyAllWindows()
