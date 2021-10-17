import pygame
import numpy as np
import keyboard
import time
from animation_functions import *
from trajectory_functions import *

def initialize_joystick():
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        return joystick

def get_axis(joystick):
    joystick.init()
    pygame.event.get()
    axis = np.array([joystick.get_axis(0), joystick.get_axis(1)])
    return axis


def controller_draw(window,canvas,joystick,pos_last,t_draw,ball_input):
    input = get_axis(joystick)
    delta_t = time() - t_draw
    delta_pos = input*delta_t*gain
    pos = pos_last + delta_pos
    t_draw = time()
    input_ball.move(pos)
    return pos, t_draw


if __name__ == "__main__":
    joystick = initialize_joystick()
    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)
    input_ball = Ball(pos, input_ball_radius, "white", animation_canvas)

    t0 = time()
    t_draw = t0
    pos_draw = window_size/2*np.array([1, 1])
    while True:
        t = time() - t0
        pos_draw, t_draw = controller_draw(animation_window,animation_canvas,joystick,pos_draw,t_draw,input_ball)

        if keyboard.is_pressed('esc'):
            print("Drawing Interupted")
            break
