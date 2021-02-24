#!/usr/bin/env python
import numpy as np
import math as m
import cv2
import threading # we use threading to run stuff asynchronously 
import time


class driver():
    def __init__(self):
        self.init = False # if the state has been initialized
        self.training_data = [] # empty list for training data
        self.Finish = False # whether we've finished recording or not
        self.Rec = False # whether we're recording or not
        self.HEIGHT = 240 #image dimensions
        self.WIDTH = 320
        self.CHANNELS = 1 #number of channels in the input image
        self.WHEELBASE = 1.56 # wheelbase of the car
        self.dt = 0 # time step
        self.cam_img = None
        # select manual control type
        self.now = time.time()

    def initialize(self, X, Y, head, speed,WB):
        self.X = X
        self.Y = Y
        self.last_X = X
        self.last_Y = Y
        self.speed = speed
        self.WB = WB
        self.init = True
        self.cam_img = None
        self.heading = 0

    def calc_head(self): # calculates heading of the car by taking inverse tangent of dy/dx (movement)
        dy = self.Y - self.last_Y
        dx = self.X - self.last_X
        self.last_Y = self.Y
        self.last_X = self.X
        if(m.fabs(dx)<0.001 and m.fabs(dy)<0.001):
            self.mh = None
        else:
            self.mh = m.atan2(dy,dx)
            # print(self.mh*57.3)

    def update_state(self,X,Y,speed,steer,heading,yawRate,time_stamp):
        self.X = X
        self.Y = Y
        self.speed = speed
        self.heading = heading
        self.yawRate = yawRate
        self.steer = steer
        self.dt = time.time()-self.now
        self.now = time.time()