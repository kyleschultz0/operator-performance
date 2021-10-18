import pandas as pd
import numpy as np
import keyboard
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command
from animation_functions import *


 

#=== variables for 2 dof ===#
L1 = 0.29
L2 = 0.22
#======#


def calculate_hebi_position(group, hebi_feedback, offset):
    theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)  
    theta = theta - np.array([1.58702857, -0.08002613])
    pos = 5000*np.array([-L1*np.sin(theta[0]) - L2*np.cos(theta[0]+theta[1]), L1*np.cos(theta[0])-L2*np.sin(theta[0]+theta[1])])
    pos[1] = animation_window_height - pos[1]
    pos += offset
    return pos

if __name__ == "__main__":
    output = []
    freq = 100 # hz
    
    group, hebi_feedback, command = initialize_hebi()
    group.feedback_frequency = freq
    group_info = group.request_info()

    if group_info is not None:
        group_info.write_gains("csv/saved_gains.xml")

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)

    theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback) 

    pos0 = calculate_hebi_position(group, hebi_feedback, offset = 0)
    offset = (window_size/2)*np.array([1, 1]) - pos0


    input_ball = Ball(calculate_hebi_position(group, hebi_feedback, offset), input_ball_radius, "white", animation_canvas)
    animation_window.update()

    t0 = time()

    while True:
        pos = calculate_hebi_position(group, hebi_feedback, offset)
        input_ball.move(pos)
        animation_window.update()

        if keyboard.is_pressed('esc'):
            print("Stopping: User input stop command")
            break





