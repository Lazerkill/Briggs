# Hexapod frame coordinate: X+ is a line from dead-center that points to HDMI ports on RPI
#                           Y+ is a line from dead-center that points to USB ports on RPI
#                           Z+ is upwards, away from ground
# Legs are organized radially around origin starting at X+ axis... 
# Leg 0 is leg 1 in MATLAB/physical robot

import busio
import board
import adafruit_pca9685
from adafruit_servokit import ServoKit
import leg_class
import numpy as np
import utilities as util
import time

class hexapod:
    def __init__(self):
        # Adafruit servo-hat classes
        i2c             = busio.I2C(board.SCL, board.SDA)
        top_hat         = adafruit_pca9685.PCA9685(i2c, address = 0x40)
        bottom_hat      = adafruit_pca9685.PCA9685(i2c, address = 0x41)
        top_kit         = ServoKit(channels=16, address = 0x40)
        bottom_kit      = ServoKit(channels=16, address = 0x41)
        
        # leg class creation
        length  = [27.0, 48.0, 153.0];
        self.legs = [None] * 6
        self.legs[0] = leg_class.leg(length, -60, bottom_kit, [12,13,14])
        self.legs[1] = leg_class.leg(length,-120, top_kit,    [12,13,14])
        self.legs[2] = leg_class.leg(length, 180, top_kit,    [ 4, 5, 6])
        self.legs[3] = leg_class.leg(length,-240, top_kit,    [ 0, 1, 2])
        self.legs[4] = leg_class.leg(length,-300, bottom_kit, [ 0, 1, 2])
        self.legs[5] = leg_class.leg(length, 360, bottom_kit, [ 4, 5, 6])
        
        # Set unique leg correction constants
        self.legs[0].setCorrection([  0,   -5, 12])
        self.legs[1].setCorrection([  0,    1,  5])
        self.legs[2].setCorrection([  0,   -5,  8])
        self.legs[3].setCorrection([  0,    3,  0])
        self.legs[4].setCorrection([  0,  -10, 15])
        self.legs[5].setCorrection([ -5,   -7, 18])
        
        # Robot is assumed to start sitting
        self.robot_status = "sitting" 
        for leg_i in self.legs: leg_i.setJoints([0, -90, 90])
        
    def stopPWM(self):
        for leg_i in self.legs: leg_i.release()
    
    def stand(self):
        if "standing" in self.robot_status:
            print("REDUNDANT COMMAND IGNORED: Robot already standing.")
            return
        
        for leg_i in self.legs: leg_i.setEE([155, 0, -45])
        time.sleep(1)
        for leg_i in self.legs: leg_i.setEE([155, 0, -138])
        time.sleep(1)
        for leg_i in self.legs: leg_i.step([75,  0,  -138], 1)
        
        self.robot_status = "standing"
        
    def sit(self):
        if "sitting" in self.robot_status:
            print("REDUNDANT COMMAND IGNORED: Robot already sitting.")
            return
        
        for leg_i in self.legs: leg_i.step([155,  0,  -138], 1)
        for leg_i in self.legs: leg_i.setEE([155, 0, -45])
        time.sleep(1)
        for leg_i in self.legs: leg_i.setJoints([0, -90, 90])
        time.sleep(1)
        for leg_i in self.legs: leg_i.release()
        
        self.robot_status = "sitting"
    
    # Handles the TIME component of a gait cycle    
    def gait(self, direction, gait_type, dist_togo = float("inf")):   
        t_stance = 2.5
        t_stride = 1.0
        t_period = t_stance + t_stride
        
        if "wave" in gait_type:
            time_vector = np.array([0, 1/6, 2/6, 3/6, 4/6, 5/6])    
        elif "ripple" in gait_type:
            time_vector = np.array([0, 1/3, 2/3,   0, 1/3, 2/3])
        elif "tripod" in gait_type:
            time_vector = np.array([0, 1/2,   0, 1/2,   0, 1/2])
            
        for leg_i in self.legs: leg_i.setNodes(direction)
            
        epoch_time = time.perf_counter()
        dist_traveled = 0
        while dist_togo > dist_traveled :
                self.robot_status = "gaiting"
                for i in range(6):
                        # Piecewise function of the time domain
                        loc = self.legs[i].getNodes()
                        t =  ((time.perf_counter() - epoch_time) + time_vector[i]*t_period) % t_period
                        if 0 <= t < t_stride:
                            if   t < t_stride*(1/3):
                                ti = 0
                                current = util.sine_interpol(loc[0,:], loc[1,:], t, t_stride/3, ti)
                            elif t < t_stride*(2/3):
                                ti = t_stride*(1/3)
                                current = util.sine_interpol(loc[1,:], loc[2,:], t, t_stride/3, ti)
                            elif t < t_stride*(3/3):
                                ti = t_stride*(2/3)
                                current = util.sine_interpol(loc[2,:], loc[3,:], t, t_stride/3, ti)
                        elif t_stride <= t < t_period:
                            ti = t_stride
                            current = util.linear_interpol(loc[3,:], loc[0,:], t, t_stance, ti)
        
                        # Transition the robot to a gait if not already in one
                        if "gaiting" not in self.legs[i].getStatus():
                            step_time = 1
                            self.legs[i].step(current, step_time)
                            self.legs[i].setStatus("gaiting")
                            epoch_time = epoch_time + step_time # retards epoch time to account for step cycle
                        else:
                            self.legs[i].setEE(current)
                # Update control loop contraints
                dist_traveled = (self.legs[1].getGait()/t_period)*(time.perf_counter() - epoch_time)
        else:
                for leg_i in self.legs: leg_i.step([75, 0, -138], 1)
                self.robot_status = "standing"
                    
