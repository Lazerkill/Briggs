# Briggs
Briggs The Hexapod. My 18 DOF hexapod. Includes MATLAB simulations and Python code. Both of which have geometry constants that can be modified to apply the code to almost any hexapod variation.

"Matlab" contains all of the files relevant to simulating the kinematics of the robot:
        
        -"Leg.m" is the class used to drive the rest of the files. Includes inverse and forward kinematic derivations.
        
        -"single_leg_FK.m" is a forward kinematics driver
        
        -"single_leg_IK.m" is an inverse kinematics driver
        
        -"single_leg_extrema.m" plots all of the possible end-effector locations for a given leg geometry
        
        -"single_leg_motion_study_interpol.m" is a driver that can be used to plan out the nodes for a gait cylce using interpolation trajectory planning
        
        -"hexapod_gait_motion_study.m" is a driver that can be used to plan out the gait for an entire hexapod 
  
"Python" contains all of the files relevant to running the hexapod with a RPI and a Adafruit servo hat:

        -"leg_class.py" is the class that contains all of the geometry definitions and servo commands for one leg

        -"hexapod_class.py" is the class that contains all of the geometry definitions and timing control for an entire robot

        -"main.py" is a driver for the hexapod class (used to issue order to the robot)
        
        -"utilities.py" is a library of math functions used for operation (includes IK, FK, and interpol functions)
