from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta
from scipy.optimize import minimize
import numpy as np

def compute_trim(mav: MavDynamics, delta: MsgDelta):
    # parameters to unput fro the trim
    # alpha, elevator, throttle
    x0 = [mav._alpha, delta.elevator, delta.throttle]
    bounds = [(0, np.deg2rad(12)), (-1, 1), (0, 1)]
    res = minimize(mav.calculate_trim_output, x0, bounds=bounds, method='SLSQP')
    #print(res)
    return(res.x[0], res.x[1], res.x[2])

def do_trim(mav, Va, alpha):
    # create initialization parameters
    delta = MsgDelta()
    Va0 = Va
    alpha0 = alpha
    beta0 = 0.
    mav.initialize_velocity(Va0, alpha0, beta0)

    # ------- set control surfaces -------------
    delta.elevator = -0.1248 #ws
    delta.aileron = 0.0 #ad
    delta.rudder = -0.0 #qe
    delta.throttle = 0.6768 #opw

    alpha, elevator, throttle = compute_trim(mav, delta)
    mav.initialize_velocity(Va0, alpha, beta0)
    delta.elevator = elevator
    delta.throttle = throttle
    return delta
