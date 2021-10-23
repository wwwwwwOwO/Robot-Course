

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
from numpy.core.numeric import NaN
from controller import Robot

from threading import Thread
import os
import sys
import time
import numpy as np
import cv2
import math
 
class PID:
    """PID Controller
    """

    def __init__(self, P=0.004, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 10.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
 


class Car():
    def __init__(self):
        self.velocity = 14
        self.COLOR_RANGE = {
            'WHITE': [(180, 0, 0), (255, 255, 255)],  # WHITE
            'BLACK': [(0, 0, 0), (90, 255, 255)],  # BLACK
        }
        self.wheels_names = ["motor1","motor2","motor3","motor4"]
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

        # GPS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.mTimeStep)


        self.mMotor = []
        self.speed1 = [0, 0, 0, 0]
        self.speed2 = [0, 0, 0, 0]
        self.maxVelocity = 0

        # 获取各传感器并激活，以mTimeStep为周期更新数值
        for i in range(4):
            self.mMotor.append(self.robot.getDevice(self.wheels_names[i]))
            self.mMotor[i].setPosition(float('inf'))
            self.mMotor[i].setVelocity(0.0)
            self.maxVelocity = self.mMotor[i].getMaxVelocity()

        self.pid = PID(P=0.008, I=0.002, D=0.002)
        

    def go(self):
        while self.robot.step(self.mTimeStep) != -1:
            for i in range(4):
                self.mMotor[i].setVelocity(self.speed_forward[i])


    def LineFollowing(self):
        self.pid.SetPoint = 320
        self.turn = 0

        # for GPS information
        self.location_x = 0.0
        self.location_y = 0.0
        while self.robot.step(self.mTimeStep) != -1:
            self.location_x, self.location_y, _ = self.gps.getValues()
            if not (math.isnan(self.location_x) or math.isnan(self.location_y)):
                break

        self.E = 0
        self.V = 0
        self.n = 0
        self.round = 0
        self.away = False
        self.start = 0
        self.end = 0


        while self.robot.step(self.mTimeStep) != -1:
            self.frame = np.frombuffer(self.camera.getImage(), np.uint8).reshape((480, 640, 4))
            self.frameLAB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2LAB)
            self.Mask = cv2.inRange(self.frameLAB, self.COLOR_RANGE['BLACK'][0], self.COLOR_RANGE['BLACK'][1])
            self.l = 0
            self.r = 0

            for i in range(self.Mask.shape[1]):
                if self.Mask[0][i] > 0:
                    self.l = i
                    for j in range(i, self.Mask.shape[1]):
                        if self.Mask[0][j] == 0:
                            self.r = j
                            break
                    break
            self.vex = (self.l + self.r) // 2

            # self.realVelocity = self.mMotor[0].getVelocity()
            # self.pid.setKp(0.01 * self.realVelocity / self.maxVelocity)
            # self.pid.setKi(0.004 * self.realVelocity / self.maxVelocity)
            # self.pid.setKd(0.004 * self.realVelocity / self.maxVelocity)
            # self.pid.setKp(0.0024 * self.realVelocity * self.realVelocity / (self.maxVelocity * self.maxVelocity))
            # self.pid.setKi(0.0012 * self.realVelocity * self.realVelocity / (self.maxVelocity * self.maxVelocity))
            # self.pid.setKd(0.0012 * self.realVelocity * self.realVelocity / (self.maxVelocity * self.maxVelocity))
            self.pid.update(self.vex)
            self.turn = self.pid.output

            # print(self.vex - 320, ":", self.turn)
            
            for i in range(4):
                self.speed1[i] = self.speed_forward[i]
                self.speed2[i] = self.speed_leftCircle[i] * self.turn

            self.speed = 0
            for i in range(4):
                self.speed = self.speed1[i] + self.speed2[i]
                if self.speed > self.maxVelocity:
                    self.speed = self.maxVelocity
                elif self.speed < -self.maxVelocity:
                    self.speed = -self.maxVelocity

                self.mMotor[i].setVelocity(self.speed)

        
            # for GPS information
            x, y, _ = self.gps.getValues()
            vel = self.mMotor[0].getVelocity()
            dist = math.sqrt(x * x + y * y)
            self.E += math.fabs(dist - 1.78)
            self.V += vel
            self.n += 1

            if math.sqrt((x - self.location_x) ** 2 + (y - self.location_y) ** 2) < 0.1 and self.away:
                self.round += 1
                self.end = time.perf_counter()
                
                print('Round ', self.round, ' 巡线误差:', self.E / self.n, ' 平均速度：', self.V / self.n, ' 耗时：', self.end - self.start)
                self.E = 0
                self.V = 0
                self.n = 0
                self.away = False
            elif math.sqrt((x - self.location_x) ** 2 + (y - self.location_y) ** 2) > 0.1:
                if not self.away:
                    self.start = time.perf_counter()
                self.away = True
                


    def LineFollowing2(self):
        while self.robot.step(self.mTimeStep) != -1:
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
                self.speed1[i] = self.speed_forward[i]
                
            if self.v1 < self.v2:
                for i in range(4):
                    self.speed2[i] = 20 * self.speed_leftCircle[i] * (self.v2 - self.v1 ) / self.Mask.shape[1]
            else:
                for i in range(4):
                    self.speed2[i] = 20 * self.speed_rightCircle[i] * (self.v1 - self.v2 ) / self.Mask.shape[1]
                    
            self.speed = 0
            for i in range(4):
                self.speed = self.speed1[i] + self.speed2[i]
                if self.speed > self.maxVelocity:
                    self.speed = self.maxVelocity
                elif self.speed < -self.maxVelocity:
                    self.speed = -self.maxVelocity

                self.mMotor[i].setVelocity(self.speed)
    
    def LineFollowing1(self):
        while self.robot.step(self.mTimeStep) != -1:
            self.frame = np.frombuffer(self.camera.getImage(), np.uint8).reshape((480, 640, 4))
            self.frameLAB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2LAB)
            self.Mask = cv2.inRange(self.frameLAB, self.COLOR_RANGE['BLACK'][0], self.COLOR_RANGE['BLACK'][1])
            self.lMask = self.Mask[100:-100, :self.Mask.shape[1]//2]
            self.rMask = self.Mask[100:-100, self.Mask.shape[1]//2:-1]
            self.larea = 0
            self.rarea = 0
            self.lcontours = cv2.findContours(self.lMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if len(self.lcontours) > 0:
                self.larea = cv2.contourArea(self.lcontours[0])
            self.rcontours = cv2.findContours(self.rMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if len(self.rcontours) > 0:
                self.rarea = cv2.contourArea(self.rcontours[0])

            for i in range(4):
                self.speed1[i] = self.speed_forward[i]

            if self.larea > self.rarea + 100:
                self.speed2[i] = self.speed_leftCircle[i]

            elif self.rarea > self.larea + 100:
                self.speed2[i] = self.speed_rightCircle[i]
                    
            self.speed = 0
            for i in range(4):
                self.speed = self.speed1[i] + self.speed2[i]
                if self.speed > self.maxVelocity:
                    self.speed = self.maxVelocity
                elif self.speed < -self.maxVelocity:
                    self.speed = -self.maxVelocity

                self.mMotor[i].setVelocity(self.speed)

            

if __name__ == '__main__':
    print(sys.executable)
    print ('Python Version {}'.format(str(sys.version).replace('\n', '')))
    car = Car()  # 初始化Car类
    # car.go()  # 运行控制器
    car.LineFollowing()

    