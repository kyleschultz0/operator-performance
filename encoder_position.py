import pandas as pd
import numpy as np
import keyboard
from encoder_functions import initialize_encoders, get_encoder_feedback
from animation_functions import *


 

#=== variables for 2 dof ===#
L1 = 0.29
L2 = 0.22
#======#


def calculate_encoder_position(arduino, offset):
    theta = get_encoder_feedback(arduino, num_encoders=2)
    pos = 2000*np.array([-L1*np.sin(theta[0]) - L2*np.cos(theta[0]+theta[1]), L1*np.cos(theta[0])-L2*np.sin(theta[0]+theta[1])])
    pos[1] = animation_window_height - pos[1]
    pos += offset
    return pos

if __name__ == "__main__":
    output = []
    arduino = initialize_encoders()
    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)

    pos0 = encoder_position(arduino, offset = 0)
    offset = (window_size/2)*np.array([1, 1]) - pos0
    input_ball = Ball(calculate_encoder_position(arduino, offset), input_ball_radius, "white", animation_canvas)
    animation_window.update()

    t0 = time()

    while True:
        pos = calculate_encoder_position(arduino, offset)
        input_ball.move(pos)
        animation_window.update()

        if keyboard.is_pressed('esc'):
            print("Stopping: User input stop command")
            break




