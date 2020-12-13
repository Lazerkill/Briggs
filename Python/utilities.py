#Utilities description: Different math functions useful for hexapod operation.
from math import sqrt, atan, atan2, acos, sin, cos, degrees, radians, pi, pow
import numpy as np

#  Input units: mm, mm
# Output units: degrees (-90/90) (array)
def inverse(ee, length):
    ee_x = ee[0]
    ee_y = ee[1]
    ee_z = ee[2]
    
    #IK-theta 1
    theta1 = atan2(ee_y, ee_x)
 
    #IK-theta 3
    numerator   = pow(sqrt(pow(ee_x, 2) + pow(ee_y, 2)) - length[0], 2) + pow(ee_z, 2) - pow(length[2], 2) - pow(length[1], 2)
    denominator = 2.0 * length[1] * length[2]
    theta3  = acos(numerator/denominator)  
 
    #IK-theta 2
    Q           = sqrt(pow(ee_x, 2) + pow(ee_y, 2)) - length[0]
    numerator   = Q*(length[2]*sin(theta3)) + ee_z*(length[2]*cos(theta3) + length[1])
    denominator = Q*(-length[2]*cos(theta3) - length[1]) + ee_z*(length[2]*sin(theta3))
    theta2      = atan(numerator/denominator)
    
    return(np.array([degrees(theta1), degrees(theta2), degrees(theta3)]))

# Input units: degree (-90/90), mm 
# Output units: mm (array)
def forward(theta, length):
    theta = [radians(i) for i in theta]
    T_01  = np.array([[cos(theta[0]),  0, -sin(theta[0]), cos(theta[0])*length[0]],
                      [sin(theta[0]),  0,  cos(theta[0]), sin(theta[0])*length[0]],
                      [            0, -1,              0,                       0],
                      [            0,  0,              0,                       1]])
    
    T_12  = np.array([[cos(theta[1]), -sin(theta[1]), 0, cos(theta[1])*length[1]],
                      [sin(theta[1]),  cos(theta[1]), 0, sin(theta[1])*length[1]],
                      [            0,              0, 1,                       0],
                      [            0,              0, 0,                       1]])

    T_23  = np.array([[cos(theta[2]), -sin(theta[2]), 0, cos(theta[2])*length[2]],
                      [sin(theta[2]),  cos(theta[2]), 0, sin(theta[2])*length[2]],
                      [            0,              0, 1,                       0],
                      [            0,              0, 0,                       1]])

    T_03  = T_01 @ T_12 @ T_23
    ee_x  = T_03[0, 3]
    ee_y  = T_03[1, 3]
    ee_z  = T_03[2, 3]
    
    return(np.array([ee_x, ee_y, ee_z]))

# Interpolates a straight line path between 2 3D coordinates over time.
# Constraints: initial and final velocities and accelerations are zero.
def sine_interpol(initial3, final3, t_curr, t_span, t_initial = 0):
    current3 = np.zeros(3)
    for i in range(3):
        initial = initial3[i]
        final  = final3[i]
        
        C = final - initial
        t = (t_curr - t_initial)/t_span
        current3[i] = C*(t - sin(2*pi*t)/(2*pi)) + initial
    return np.array(current3)

# Interpolates a straight line path between 2 3D coordinates over time.
# Constraints: constant velocity over trajectory.
def linear_interpol(initial3, final3, t_curr, t_span, t_initial = 0):
    current3 = np.zeros(3)
    for i in range(3):
        initial = initial3[i]
        final  = final3[i]
        
        t_final     = t_span + t_initial
        numerator   = final - initial
        denominator = t_final - t_initial
        current3[i] = (numerator/denominator)*(t_curr - t_initial) + initial
    return np.array(current3)

# Linearly maps the value from a known domain to a known range.
# (Behavior is idential to the "map()" function in Arduino IDE)
def bound(input_val, input_min, input_max, output_min, output_max):
    answer = (input_val - input_min) * (output_max - output_min) / (input_max - input_min) + output_min
    if answer > output_max:
        return output_max
    if answer < output_min:
        return output_min
    else:
        return answer    