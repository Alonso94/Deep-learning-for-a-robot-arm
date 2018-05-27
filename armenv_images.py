import time
import vrep
import sys
import numpy as np
import cv2
import math
import tensorflow as tf

class ArmEnv(object):
    action_bound = [-1, 1]
    action_dim = 7
    state_dim = (128, 128, 1)
    dt = .1  # refresh rate
    done = False
    point_l = 15
    grab_counter = 0
    gl = (29, 86, 6)
    gu = (64, 255, 255)
    h = []
    a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    od1 = od2 = od3 = 0
    x1 = x2 = x3 = x4 = x5 = x6 = 0
    y1 = y2 = y3 = y4 = y5 = y6 = 0
    d1 = d2 = d3 = 0
    observation_space = np.array([128, 128 ,1])

    def __init__(self, mode='easy'):
        self.a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vrep.simxFinish(-1)
        self.ID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.ID != -1:
            print 'Connected'
        else:
            print 'error'
            sys.exit('Error')
        r1, self.cam = vrep.simxGetObjectHandle(self.ID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        r2, self.cam1 = vrep.simxGetObjectHandle(self.ID, 'Vision_sensor0', vrep.simx_opmode_oneshot_wait)
        r3, self.cam2 = vrep.simxGetObjectHandle(self.ID, 'Vision_sensor1', vrep.simx_opmode_oneshot_wait)
        j = []
        for i in range(7):
            j.append(
                vrep.simxGetObjectHandle(self.ID, ("redundantRob_joint%d" % (i + 1)), vrep.simx_opmode_oneshot_wait))
        for (x, y) in j:
            self.h.append(y)
        for i in range(7):
            vrep.simxSetJointPosition(self.ID, self.h[i], self.a[i], vrep.simx_opmode_oneshot)

    def step(self, action):

        # action = (node1 angular v, node2 angular v)
        action = np.clip(action, *self.action_bound)
        if len(action) == 7:
            for i in range(7):
                self.a[i] += action[i] * self.dt
                self.a[i] %= np.pi
                p = -np.pi / 2 + self.a[i]
                vrep.simxSetJointPosition(self.ID, self.h[i], p, vrep.simx_opmode_oneshot)
        ec, res, im = vrep.simxGetVisionSensorImage(self.ID, self.cam, 0, vrep.simx_opmode_streaming)
        img1 = np.array(im, dtype=np.uint8)
        img1.resize([128, 128, 3])
        ec, res, im = vrep.simxGetVisionSensorImage(self.ID, self.cam1, 0, vrep.simx_opmode_streaming)
        img2 = np.array(im, dtype=np.uint8)
        img2.resize([128, 128, 3])
        ec, res, im = vrep.simxGetVisionSensorImage(self.ID, self.cam2, 0, vrep.simx_opmode_streaming)
        img3 = np.array(im, dtype=np.uint8)
        img3.resize([128, 128, 3])
        hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.gl, self.gu)
        # mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.gl, self.gu)
        # mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts1 = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        hsv = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.gl, self.gu)
        # mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts2 = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) == 2:
            M = cv2.moments(cnts[0])
            self.x1 = int(M["m10"] / M["m00"])
            self.y1 = int(M["m01"] / M["m00"])
            M = cv2.moments(cnts[1])
            self.x2 = int(M["m10"] / M["m00"])
            self.y2 = int(M["m01"] / M["m00"])
            self.d1 = np.sqrt(np.square(self.x1 - self.x2) + np.square(self.y1 - self.y2))
        else:
            self.d1 = self.od1
        if len(cnts1) == 2:
            M = cv2.moments(cnts1[0])
            self.x3 = int(M["m10"] / M["m00"])
            self.y3 = int(M["m01"] / M["m00"])
            M = cv2.moments(cnts1[1])
            self.x4 = int(M["m10"] / M["m00"])
            self.y4 = int(M["m01"] / M["m00"])
            self.d2 = np.sqrt(np.square(self.x4 - self.x3) + np.square(self.y3 - self.y4))
        else:
            self.d2 = self.od2
        if len(cnts2) == 2:
            M = cv2.moments(cnts2[0])
            self.x5 = int(M["m10"] / M["m00"])
            self.y5 = int(M["m01"] / M["m00"])
            M = cv2.moments(cnts2[1])
            self.x6 = int(M["m10"] / M["m00"])
            self.y6 = int(M["m01"] / M["m00"])
            self.d3 = np.sqrt(np.square(self.x6 - self.x5) + np.square(self.y6 - self.y5))
        else:
            self.d3 = self.od3
        # s = np.hstack(
        #    [self.a[0], self.a[1], self.a[2], self.a[3], self.a[4], self.a[5], self.a[6], self.a[7], self.x1, self.y1,
        #     self.x2, self.y2, self.x3, self.y3, self.x4, self.y4, self.x5, self.y5, self.x6, self.y6])
        r, self.done = self._r_func(self.d1, self.d2, self.d3)
        #img1.resize([32, 32, 3])
        img1 = tf.image.rgb_to_grayscale(img1)
        with tf.Session():
            img1=img1.eval()
        #print img1.shape
        #img = np.array(img1)
        return img1, r, self.done, {}

    def reset(self):
        self.a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.done = False
        for i in range(7):
            vrep.simxSetJointPosition(self.ID, self.h[i], self.a[i], vrep.simx_opmode_oneshot)
        r, p = vrep.simxGetObjectHandle(self.ID, 'Cuboid', vrep.simx_opmode_oneshot_wait)
        r, b = vrep.simxGetObjectPosition(self.ID, p, self.cam, vrep.simx_opmode_oneshot_wait)
        b[2] = np.random.uniform(0.8, 1.8)
        vrep.simxSetObjectPosition(self.ID, p, self.cam, b, vrep.simx_opmode_oneshot)
        s, r, done, info = self.step(self.a)
        return s

    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self, d1, d2, d3):
        done = False
        # - np.log10(np.square(d1)
        r = 0
        if d1 - self.od1 == 0 and d2 - self.od2 == 0 and d3 - self.od3 == 0:
            r -= 30
        r -= 0.1 * (d1 - self.od1)
        r -= (0.1 * (d2 - self.od2))
        r -= (0.1 * (d3 - self.od3))
        self.od1 = d1
        self.od2 = d2
        self.od3 = d3
        if d1 < 20 or d2 < 20 or d3 < 20:
            r += 30
        if d1 < 20 and d2 < 20 and d3 < 20:
            r += 10000
            done = True
        return r, done
