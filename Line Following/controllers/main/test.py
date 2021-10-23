

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# create the Robot instance.
#robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
#while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.

# -*- coding:UTF-8 -*-
from controller import Robot

import os
import sys
import time
import numpy as np
import cv2

class Car():
    def __init__(self):
        self.velocity = 10
        self.COLOR_RANGE = {
            'WHITE': [(180, 0, 0), (255, 255, 255)],  # WHITE
            'BLACK': [(0, 0, 0), (100, 255, 255)],  # BLACK
        }
        self.wheels_names = ['motor1','motor2','motor3','motor4']
        self.speed_forward =[self.velocity, self.velocity ,self.velocity ,self.velocity]
        self.speed_backward =[-self.velocity, -self.velocity ,-self.velocity ,-self.velocity]
        self.speed_leftward =[self.velocity, -self.velocity ,self.velocity ,-self.velocity]
        self.speed_rightward =[-self.velocity, self.velocity ,-self.velocity ,self.velocity]
        self.speed_leftCircle =[self.velocity, -self.velocity ,-self.velocity ,self.velocity]
        self.speed_rightCircle =[-self.velocity, self.velocity ,self.velocity ,-self.velocity]
        self.robot = Robot()  # 初始化Robot类以控制机器人
        self.mTimeStep = int(self.robot.getBasicTimeStep())  # 获取当前每一个仿真步所仿真时间mTimeStep

        # vision
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.mTimeStep)
        self.width = self.camera.getWidth() # 640
        self.height = self.camera.getHeight() # 480
        self.frame = None # 

        self.mMotor = []
        self.speed1 = [0, 0, 0, 0]
        self.speed2 = [0, 0, 0, 0]
        # 获取各传感器并激活，以mTimeStep为周期更新数值
        for i in range(4):
            self.mMotor.append(self.robot.getDevice(self.wheels_names[i]))
            self.mMotor[i].setPosition(float('inf'))
            self.mMotor[i].setVelocity(0.0)
        
    def go(self):
        while self.robot.step(self.mTimeStep) != -1:
            for i in range(4):
                self.mMotor[i].setVelocity(self.speed_forward[i])

    def LineFollowing(self):
        while self.robot.step(self.mTimeStep*4) != -1:
            self.frame = np.frombuffer(self.camera.getImage(), np.uint8).reshape((480, 640, 4))
            self.frameLAB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2LAB)
            self.Mask = cv2.inRange(self.frameLAB, self.COLOR_RANGE['BLACK'][0], self.COLOR_RANGE['BLACK'][1])
            self.l = 0
            self.r = 0
            for i in range(self.Mask.shape[1]):
                if self.Mask[0][i]:
                    self.l = i
                    for j in range(i, self.Mask.shape[1]):
                        if self.Mask[0][j] == 0:
                            self.r = j
                        break
                    break
            self.v1 = (self.l + self.r) // 2
            
            self.l = 0
            self.r = 0
            for i in range(self.Mask.shape[1]):
                if self.Mask[self.Mask.shape[0]-1][i]:
                    self.l = i
                    for j in range(i, self.Mask.shape[1]):
                        if self.Mask[self.Mask.shape[0]-1][j] == 0:
                            self.r = j
                        break
                    break
            self.v2 = (self.l + self.r) // 2
            
            print(self.v1, ":", self.v2)
            
            for i in range(4):
                self.speed1[i] = self.speed_forward[i] / 2
                
            if self.v1 < self.v2:
                for i in range(4):
                    self.speed1[i] += 20*self.speed_leftCircle[i] * (self.v2 - self.v1 ) / self.Mask.shape[1]
            else:
                for i in range(4):
                    self.speed1[i] += 20*self.speed_rightCircle[i] * (self.v1 - self.v2 ) / self.Mask.shape[1]
                    
            for i in range(4):
                self.mMotor[i].setVelocity(self.speed1[i])


if __name__ == '__main__':
    print(sys.executable)
    print ('Python Version {}'.format(str(sys.version).replace('\n', '')))
    car = Car()  # 初始化Walk类
    car.LineFollowing()
    # car.go()  # 运行控制器
