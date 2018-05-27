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
    state_dim = 10
    dt = .1  # refresh rate
    done = False
    a = [1650, 2000, 750, 1200, 1200, 2425]
    lp = [1600, 1800, 800, 500, 700, 700, 700]
    up = [1900, 2200, 1800, 2500, 2400, 2400, 2400]
    x1 = x2 = x3 = x4 = 0
    v = 0
    final = 100
    counter = 0

    def __init__(self, mode='easy'):
        self.counter = 0
        self.a = [1650, 2000, 750, 1200, 1200, 2425]
        self.mid1 = (self.x1 + self.x2) / 2
        self.mid2 = (self.x3 + self.x4) / 2
        self.l1 = self.x2 - self.x1
        self.l2 = self.x4 - self.x3
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        if self.ser.writable():
            print 'Connected'
        else:
            print 'error'
            sys.exit('Error')
        self.cam = cv2.VideoCapture(1)
        self.cam.set(3, 120)
        self.cam.set(4, 160)

    def step(self, action):
        # action = (node1 angular v, node2 angular v)
        action = np.clip(action, *self.action_bound)
        if len(action) == 6:
            for i in range(6):
                if action[i] < -0.25:
                    c = -15
                elif action[i] > 0.25:
                    c = 15
                else:
                    c = 0
                if i == 0 and c == 15:
                    self.a[0] += 5
                elif self.up[i] > (self.a[i] + c) > self.lp[i]:
                    self.a[i] += c
                self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                sleep(.1)
        (g, f) = self.cam.read()
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, 100)
        cv2.imshow('f', mask)
        cv2.waitKey(1)
        self.x1 = self.x3 = 200
        self.x2 = self.x4 = 0
        # print mask[0][0]
        for i in range(120):
            if mask[0][i] == 255:
                self.x1 = min(self.x1, i)
                self.x2 = max(self.x2, i)
            if mask[119][i] == 255:
                self.x3 = min(self.x3, i)
                self.x4 = max(self.x4, i)
        s = np.hstack([self.a[0], self.a[1], self.a[2], self.a[3], self.a[4], self.a[5],
                       self.x1, self.x2, self.x3, self.x4])
        r, self.done = self._r_func(self.x1, self.x2, self.x3, self.x4)
        return s, r, self.done, {}

    def reset(self):
        w = [1650, 2000, 750, 1200, 1200, 2425]
        self.done = False
        for i in range(6):
            if self.a[i] < w[i]:
                for self.a[i] in range(self.a[i], w[i], 15):
                    self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                    sleep(.1)
            else:
                for self.a[i] in range(self.a[i], w[i], -15):
                    self.ser.write(str(chr(i + 97)) + str(self.a[i]))
                    sleep(.1)
        self.v = w[0]
        self.final = w[0] + 200
        (g, f) = self.cam.read()
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, 100)
        self.x1 = self.x3 = 200
        self.x2 = self.x4 = 0

        # print mask[0][0]
        for i in range(120):
            if mask[0][i] == 255:
                self.x1 = min(self.x1, i)
                self.x2 = max(self.x2, i)
            if mask[119][i] == 255:
                self.x3 = min(self.x3, i)
                self.x4 = max(self.x4, i)
        self.mid1 = 79
        self.mid2 = 79
        self.l1 = self.x2 - self.x1
        self.l2 = self.x4 - self.x3
        s = np.hstack([self.a[0], self.a[1], self.a[2], self.a[3], self.a[4], self.a[5],
                       self.x1, self.x2, self.x3, self.x4])
        return s

    def _r_func(self, x1, x2, x3, x4):
        done = False
        if x2 == 200 or x4 == 200:
            return -10, True
        mid1 = (x1 + x2) / 2
        mid2 = (x3 + x4) / 2
        l1 = x2 - x1
        l2 = x4 - x3
        if l1 < (self.l1 - 5) or l1 > (self.l1 + 5):
            return -10, True
        if l2 < (self.l2 - 5) or l2 > (self.l2 + 5):
            return -10, True
        d1 = np.square(mid1 - mid2)
        d2 = np.square(mid1 - self.mid1)
        d3 = np.square(mid2 - self.mid2)
        d4 = np.square(l1 - self.l1)
        d5 = np.square(l2 - self.l2)
        r = -0.001 * (d1 + 3 * (d2 + d3) + 10 * (d4 + d5))
        r += 0.1 * (self.a[0] - self.v)
        if self.a[0] >= self.final:
            r += 10
            done = True
        return r, done
