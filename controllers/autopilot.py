"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
# from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pid_control import PIDControl
from controllers.pd_control import PDControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from models.mav_dynamics_control import MavDynamics


airspeed_throttle_kp = 0.05
airspeed_throttle_ki = 0.05

yaw_damper_kp = 10.0
yaw_damper_kd = 1.0

alpha_elevator_kp = -20
alpha_elevator_ki = -20
alpha_elevator_kd = -2

ALT_kp = 0.05
ALT_ki = 0.01
ALT_kd = 0.0001

gamma_kp = 1.2
gamma_ki = 1
gamma_kd = 0.001

chi_kp = 0.5
chi_ki = 0.01
chi_kd = 0.05

roll_kp = 0.21 #.22
roll_ki = 0.04 #.02
roll_kd = 0.01 #.01


class Autopilot:
    def __init__(self, delta, mav: MavDynamics, ts_control):
        
        self.throttle_from_airspeed = PIControl(
            kp=airspeed_throttle_kp,
            ki=airspeed_throttle_ki,
            Ts=ts_control,
            min=0.0,
            max=1.0,
            init_integrator = delta.throttle/airspeed_throttle_ki
            )
        
        self.elevator_from_alpha = PIDControl(
            kp=alpha_elevator_kp,
            ki=alpha_elevator_ki,
            kd=alpha_elevator_kd,
            min=-1,
            max=1,
            Ts = ts_control,
            init_integrator=delta.elevator/alpha_elevator_ki
            )
        
        self.yaw_damper = PDControl(
            kp=yaw_damper_kp, 
            kd=yaw_damper_kd, 
            Ts=ts_control, 
            min=-1,
            max=1
            )
        
        self.ALT_controller = PIDControl(
            kp=ALT_kp,
            ki=ALT_ki,
            kd=ALT_kd,
            min=np.deg2rad(-15),
            max=np.deg2rad(15),
            Ts=ts_control,
            init_integrator=0.0
        )
        
        self.gamma_controller = PIDControl(
            kp=gamma_kp,
            ki=gamma_ki,
            kd=gamma_kd,
            min=np.deg2rad(-2),
            max=np.deg2rad(12),
            Ts=ts_control,
            init_integrator=mav.true_state.alpha/gamma_ki,
        )
        
        self.chi_controller = PIDControl(
            kp= chi_kp,
            ki= chi_ki,
            kd = chi_kd,
            Ts=ts_control,
            min=np.deg2rad(-45),
            max=np.deg2rad(45),
            init_integrator=0.0
        )
        
        self.roll_controller = PIDControl(
            kp= roll_kp,
            ki= roll_ki,
            kd = roll_kd,
            Ts=ts_control,
            min=-1.0,
            max=1.0,
            init_integrator=delta.aileron/roll_ki
        )
            

        # # instantiate lateral-directional controllers
        # self.roll_from_aileron = PDControlWithRate(
        #                 kp=AP.roll_kp,
        #                 kd=AP.roll_kd,
        #                 limit=np.radians(45))
        # self.course_from_roll = PIControl(
        #                 kp=AP.course_kp,
        #                 ki=AP.course_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        # # self.yaw_damper = TransferFunction(
        # #                 num=np.array([[AP.yaw_damper_kr, 0]]),
        # #                 den=np.array([[1, AP.yaw_damper_p_wo]]),
        # #                 Ts=ts_control)
        # self.yaw_damper = TFControl(
        #                 k=AP.yaw_damper_kr,
        #                 n0=0.0,
        #                 n1=1.0,
        #                 d0=AP.yaw_damper_p_wo,
        #                 d1=1,
        #                 Ts=ts_control)

        # # instantiate longitudinal controllers
        # self.pitch_from_elevator = PDControlWithRate(
        #                 kp=AP.pitch_kp,
        #                 kd=AP.pitch_kd,
        #                 limit=np.radians(45))
        # self.altitude_from_pitch = PIControl(
        #                 kp=AP.altitude_kp,
        #                 ki=AP.altitude_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        # self.airspeed_from_throttle = PIControl(
        #                 kp=AP.airspeed_throttle_kp,
        #                 ki=AP.airspeed_throttle_ki,
        #                 Ts=ts_control,
        #                 limit=1.0)
        self.commanded_state = MsgState()

    def update(self, cmd, state, reset=False):
        delta = MsgDelta(elevator=0,
            aileron=0,
            rudder=0,
            throttle=0)
	#### TODO #####
        # lateral autopilot
        delta.rudder = self.yaw_damper.update(0, state.beta)

        # longitudinal autopilot
        delta.throttle = self.throttle_from_airspeed.update(cmd.airspeed_command, state.Va)
        
        
        #Alpha loop
        cmd_gamma = self.ALT_controller.update(cmd.altitude_command, state.altitude, reset_flag=reset)
        cmd_alpha = self.gamma_controller.update(cmd_gamma, state.gamma, reset_flag=reset)
        delta.elevator = self.elevator_from_alpha.update(cmd_alpha , state.alpha)
        # construct control outputs and commanded states
        
        # roll loop 
        
        cmd_roll = self.chi_controller.update(cmd.course_command, state.chi)
        delta.aileron = self.roll_controller.update(cmd_roll, state.phi)
        # delta.aileron = self.roll_controller.update(cmd.course_command, state.phi)

        
        
        # self.commanded_state.altitude = 0
        # self.commanded_state.Va = 0
        # self.commanded_state.phi = 0
        # self.commanded_state.theta = 0
        # self.commanded_state.chi = 0
        
        self.commanded_state.gamma = cmd_gamma
        self.commanded_state.alpha = cmd_alpha
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.theta = 0
        
        self.commanded_state.chi = cmd.course_command
        
        self.commanded_state.phi = cmd_roll
        # self.commanded_state.phi = cmd.course_command


        

        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
