"""
msg_autopilot
    - messages type for input to the autopilot
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""
import numpy as np



class MsgAutopilot:
    def __init__(self):
        # Base Config
        self.airspeed_command = 25.0  # commanded airspeed m/s
        self.course_command = np.deg2rad(0)  # commanded course angle in rad
        self.altitude_command = 100.0  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle

        # Test1
        # self.airspeed_command = 25.0  # commanded airspeed m/s
        # self.course_command = np.deg2rad(90)  # commanded course angle in rad
        # self.altitude_command = 150.0  # commanded altitude in m
        # self.phi_feedforward = 0.0  # feedforward command for roll angle
        
        # Test2
        # self.airspeed_command = 25.0  # commanded airspeed m/s
        # self.course_command = np.deg2rad(0)  # commanded course angle in rad
        # self.altitude_command = 300.0  # commanded altitude in m
        # self.phi_feedforward = 0.0  # feedforward command for roll angle
        
        # Test3
        # self.airspeed_command = 30.0  # commanded airspeed m/s
        # self.course_command = np.deg2rad(45)  # commanded course angle in rad
        # self.altitude_command = 150.0  # commanded altitude in m
        # self.phi_feedforward = 0.0  # feedforward command for roll angle
        
        # Test4
        # self.airspeed_command = 30.0  # commanded airspeed m/s
        # self.course_command = np.deg2rad(175)  # commanded course angle in rad
        # self.altitude_command = 600.0  # commanded altitude in m
        # self.phi_feedforward = 0.0  # feedforward command for roll angle

