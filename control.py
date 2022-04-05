from trajectory_functions import Trajectory
from operation import loop_timer, reset_timer


window_size = 1000
oneD = False
trajectory_type = "sines"
trial = 1
f = 0.025 # default: 0.025
T = 120

def optimal_control(gain, trajectory, t):
    uinv = trajectory.veld(t)/gain
    return uinv