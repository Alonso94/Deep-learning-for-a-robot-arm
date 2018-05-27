import time
import vrep
import sys
import numpy as np
import cv2
import math
import sys
from time import sleep

sys.path.insert(0, u'/usr/local/lib/python2.7/dist-packages')
import serial


class ArmEnv(object):
    action_bound = [-1, 1]
    action_dim = 6
    state_dim = 11
    dt = .1  # refresh rate
    done = False
    point_l = 15
    grab_counter = 0
    gl = (29, 100, 35)
    gu = (85, 255, 255)
    h = []
    a = [1700, 1400, 1300]#, 1500, 1500, 1500, 1500]
    lp = [1200, 1200, 800]#, 500, 500, 500, 700]
    up = [2200, 1600, 1800]#, 2500, 2500, 2500, 2300]
    od1 = od2 = od3 = 0
    x1 = x2 = x3 = x4 = x5 = x6 = 0
    y1 = y2 = y3 = y4 = y5 = y6 = 0
    d1 = d2 = d3 = 0
    counter =0

    def __init__(self, mode='easy'):
        self.a = [1700, 1500, 1300]
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        if self.ser.writable():
            print 'Connected'
        else:
            print 'error'
            sys.exit('Error')
        self.cam1 = cv2.VideoCapture(1)
        self.cam2 = cv2.VideoCapture(2)
        self.cam1.set(3, 160)
        self.cam1.set(4, 120)
        self.cam2.set(3, 160)
        self.cam2.set(4, 120)

    def step(self, action):
        # action = (node1 angular v, node2 angular v)
        action = np.clip(action, *self.action_bound)

        if len(action) == 6:
            for i in range(3):
                if action[2 * i] < action[2 * i + 1] and (self.a[i] + 15) < self.up[i]:
                    self.a[i] += 15
                if action[2 * i] >= action[2 * i + 1] and (self.a[i] - 15) > self.lp[i]:
                    self.a[i] -= 15
                self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                sleep(.1)
        (g, img1) = self.cam1.read()
        (g, img2) = self.cam2.read()
        hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.gl, self.gu)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.gl, self.gu)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)
        cv2.waitKey(1)
        cnts1 = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
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
        s = np.hstack(
            [self.a[0], self.a[1], self.a[2], self.x1, self.y1,
             self.x2, self.y2, self.x3, self.y3, self.x4, self.y4])
        r, self.done = self._r_func(self.d1, self.d2)
        return s, r, self.done, {}

    def reset(self):
        w= [1700, 1500, 1300]
        self.done = False
        for i in range(3):
            if self.a[i] < w[i]:
                for self.a[i] in range(self.a[i], w[i], 15):
                    self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                    sleep(.025)
            else:
                for self.a[i] in range(self.a[i], w[i], -15):
                    self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                    sleep(.025)
        self.counter+=1
        if self.counter%100==0:
            print 'change position'
            input('x')
        k=[]
        for i in range(6):
            k.append(0)
        s, r, done, info = self.step(k)
        return s

    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self, d1, d2):
        done = False
        # - np.log10(np.square(d1)
        r=0
        r -= (0.0001 * np.square(d1))
        r -= (0.0001 * np.square(d2))
        self.od1 = d1
        self.od2 = d2
        if d1 < 25 and d2 < 25 :
            r += 10000
            done = True
        return r, done
