#Leg frame coordinate: X+ is a line from the base of the leg towards EE
#                      Y+ is a line from the base of the leg to the right of EE
#                      Z+ is upwards, away from ground
import utilities as util
import numpy as np
import time

class leg:
    def __init__(self, length, radial_offset, servo_hat, pins):
        # Design and electrical variables
        self.length         = np.array(length)
        self.radial_offset  = radial_offset
        self.servo_hat      = servo_hat    
        self.pins           = pins # Input: A list of 3 int
    
        # Command and control variables
        self.EE_ideal    = None
        self.theta_ideal = np.array([0, -90, 90]) #robot is assumed to start "sitting"
        self.theta_real  = None
        self.correction  = np.array([0,   0,  0]) #leg is initially assumed to be ideal 
        self.nodes       = np.full((4, 3), None)
        self.gait_length = None
        self.gait_height = None
        self.leg_status  = "sitting"
        
    def setStatus(self, new_status):
        self.leg_status = new_status
    def getStatus(self):
        return self.leg_status
    
    # Clears PWM signal to servos (power-down the leg)
    def release(self): 
        for i in range(3): self.servo_hat.servo[self.pins[i]].angle = None
    
    # Sets the theta offset values which correct servo innacuracies
    def setCorrection(self, new_correction):
        self.correction = np.array(new_correction)
    
    # Actuate leg by manually setting joint angles
    def setJoints(self, theta_desired): 
        self.theta_ideal = np.array(theta_desired)
        self.EE_ideal    = util.forward(self.theta_ideal, self.length)
        # Correct for servo inaccuracies 
        self.theta_real  = self.theta_ideal + self.correction
        # Converts theta from "Matlab-space" to "servo-ready-space"
        self.theta_real[0] = util.bound(      self.theta_real[0], -90,  90, 0, 180)
        self.theta_real[1] = util.bound(      self.theta_real[1], -90,  90, 0, 180)
        self.theta_real[2] = util.bound(180 - self.theta_real[2],   0, 180, 0, 180)
        # Send commands to corresponding servo hat and pin
        for i in range(3): self.servo_hat.servo[self.pins[i]].angle = self.theta_real[i]
    def getJoints(self):
        if self.theta_real is not None:
            return self.theta_real
        else:
            self.theta_real = self.theta_ideal + self.correction
            return self.theta_real
    
    # Actuate leg by choosing an end-effector coordinate (XYZ)
    def setEE(self, EE_desired): 
        self.theta_ideal = util.inverse(EE_desired, self.length)
        # Validate kinematic solution
        if not np.allclose(EE_desired, util.forward(self.theta_ideal, self.length)):
            print("OUT-OF-BOUNDS COMMAND IGNORED: Kinematic solution is not real.")
        else:
            self.setJoints(self.theta_ideal)    
    def getEE(self):
        if self.EE_ideal is not None:
            return self.EE_ideal
        else:
            return util.forward(self.theta_ideal, self.length)

# =============================================================================
#                          DERIVED TRAJECTORY FUNCTIONS
# Uses the previous actuator commands to accomplish non-trivial manipulator changes.
# Time is a function of manipulator movement.
# =============================================================================
    
    #Description: Moves the leg to a desired location over set time
    #Note: Latching function. Loops until elapsed time or error.
    def stance(self, EE_desired, t_span): 
        self.leg_status = "moving"
        time_initial        = time.perf_counter()    
        EE_initial          = self.EE_ideal
        while (time.perf_counter() - time_initial) < t_span:
            t_curr = time.perf_counter()
            self.setEE(util.linear_interpol(EE_initial, EE_desired, t_curr, time_initial))   
        self.leg_status = "static"
    
    #Description: Moves the leg to a desired location WITHOUT dragging
    #Note: Latching function. Loops until elapsed time or error.
    def step(self, EE_desired, t_span): 
        step_height = 30
        self.leg_status = "moving"
        # Move to Z+step height above current location
        time_initial        = time.perf_counter()
        EE_initial          = self.EE_ideal
        interpol_target     = self.EE_ideal
        interpol_target[2]  = interpol_target[2] + step_height
        while (time.perf_counter() - time_initial) < t_span*(1/3):
            t_curr = time.perf_counter()
            self.setEE(util.sine_interpol(EE_initial, interpol_target, t_curr, t_span/3, time_initial))
        # Move to Z+step height above desired location
        time_initial        = time.perf_counter()
        EE_initial          = self.EE_ideal
        interpol_target     = EE_desired
        interpol_target[2]  = interpol_target[2] + step_height
        while (time.perf_counter() - time_initial) < t_span*(1/3):
            t_curr = time.perf_counter()
            self.setEE(util.sine_interpol(EE_initial, interpol_target, t_curr, t_span/3, time_initial))
        # Move to desired location
        time_initial        = time.perf_counter()    
        EE_initial          = self.EE_ideal
        interpol_target     = EE_desired
        interpol_target[2]  = interpol_target[2] - step_height
        while (time.perf_counter() - time_initial) < t_span*(1/3):
            t_curr = time.perf_counter()
            self.setEE(util.sine_interpol(EE_initial, EE_desired, t_curr, t_span/3, time_initial))
        self.leg_status = "static"
        
    #Description: Creates gait nodes to move in desired direction wrt X+ in robot frame    
    #Note: Non-latching function.
    def setNodes(self, desired_direction): 
        #bottom center of the gait cycle as an offset from ee neutral position
        gait_x0 = 35
        gait_z0 = 15
        naught  = np.array([ self.length[0]+self.length[1]+gait_x0, 0, -self.length[2]+gait_z0])

        #lengths of the rectangle which defines the gait and direction of travel
        delta_y   = 55
        delta_z   = 30
        self.gait_length = delta_y
        self.gait_height = delta_z
        
        # Coordinates of the gait in the ground frame
        #        3---------dy--------2
        #        |                   |
        #        dz                  dz
        #        |                   |
        #        4---------0---------1    
        corrected_direction = np.radians(270 - self.radial_offset - desired_direction);
        rotz = np.array([
                        [np.cos(corrected_direction), -np.sin(corrected_direction), 0],
                        [np.sin(corrected_direction),  np.cos(corrected_direction), 0],
                        [                          0,                            0, 1]
                        ])
        delta = rotz @ np.array([[0], [delta_y], [delta_z]])
        self.nodes[0, :] = [naught[0] - delta[0], naught[1] - delta[1], naught[2]]
        self.nodes[1, :] = [naught[0] - delta[0], naught[1] - delta[1], naught[2] + delta[2]]
        self.nodes[2, :] = [naught[0] + delta[0], naught[1] + delta[1], naught[2] + delta[2]]
        self.nodes[3, :] = [naught[0] + delta[0], naught[1] + delta[1], naught[2]]  
    def getNodes(self):
        return np.array(self.nodes)
    def getGait(self):
        return self.gait_length


        
        
        
       
        
