import pandas as pd
import numpy as np
import keyboard
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command
from animation_functions import *

# Initialize initializing controller 1 and 2 integral error sum:
init1_e1 = 0
init1_e2 = 0
init2_e1 = 0
init2_e2 = 0
init3_e1 = 0
init3_e2 = 0
init_tol1 = 0.15 # rad
init_tol2 = 0.01 # rad
init_tol3 = 0.02 # rad
 
# Initialize loop timer previous time:
t_1 = 0

#=== variables for 2 dof ===#
L1 = 0.285
L2 = 0.265
#======#

workspace_size = 0.39

type = "hebi"

def calculate_hebi_position(group, hebi_feedback, offset):
    pos_scale = window_size/workspace_size
    theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)
    pos = pos_scale*np.array([-L1*np.sin(theta[0]) - L2*np.cos(theta[0]+theta[1]), L1*np.cos(theta[0])-L2*np.sin(theta[0]+theta[1])])
    pos[1] = animation_window_height - pos[1]
    pos += offset
    return pos

# Functions for initial position ========================================================================

def reset_timer():
    global t_1
    t_1 = 0
    t0 = time()
    t = 0
    return t, t0

def loop_timer(t0, T, print_loop_time=False):
    global t_1
    t = time()-t0
    while (t - t_1) < T:
        t = time()-t0
    if print_loop_time:
        print('Loop Time:', round(t-t_1, 8), 'seconds')
    t_1 = t
    return t

def initializing_controller1(theta,theta_d,omega,omega_d,tol,slack):
    global init1_e1
    global init1_e2
    theta_d = theta_d - slack*np.array([1, 1])
    # Gains with cables:
    kp1 = 1.0
    kd1 = 0.055
    ki1 = 0.005
    kp2 = 1.0
    kd2 = 0.055
    ki2 = 0.005
    init1_e1 += theta_d[0]-theta[0]
    init1_e2 += theta_d[1]-theta[1]
    effort = np.array([kp1*(theta_d[0]-theta[0]) + kd1*(omega_d[0]-omega[0]) + ki1*init1_e1,
                       kp2*(theta_d[1]-theta[1]) + kd2*(omega_d[1]-omega[1]) + ki2*init1_e2])
    if abs(theta_d[0]-theta[0]) < tol and abs(theta_d[1]-theta[1]) < tol:
        return effort, True
    else:
        return effort, False
    
def initializing_controller2(theta,theta_d,tol,slack):
    global init2_e1
    global init2_e2
    theta_d = theta_d - slack*np.array([1, 1])
    # Gains with cables:
    kp1 = 2.0
    ki1 = 0.2
    kp2 = 2.0
    ki2 = 0.2
    init2_e1 += theta_d[0]-theta[0]
    init2_e2 += theta_d[1]-theta[1]
    effort = np.array([kp1*(theta_d[0]-theta[0]) + ki1*init2_e1,
                       kp2*(theta_d[1]-theta[1]) + ki2*init2_e2])
    if abs(theta_d[0]-theta[0]) < tol and abs(theta_d[1]-theta[1]) < tol:
        return effort, True
    else:
        return effort, False

def initializing_controller3(theta,theta_d,omega,omega_d,tol):
    global init3_e1
    global init3_e2
    # Gains with cables:
    kp1 = 0.75
    ki1 = 0.01
    kd1 = 0.2
    kp2 = 0.75
    ki2 = 0.01
    kd2 = 0.2
    init3_e1 += theta_d[0]-theta[0]
    init3_e2 += theta_d[1]-theta[1]
    effort = np.array([kp1*(theta_d[0]-theta[0]) + kd1*(omega_d[0]-omega[0]) + ki1*init3_e1,
                       kp2*(theta_d[1]-theta[1]) + kd2*(omega_d[1]-omega[1]) + ki2*init3_e2])
    if abs(theta_d[0]-theta[0]) < tol and abs(theta_d[1]-theta[1]) < tol:
        return effort, True
    else:
        return effort, False

def set_hebi_position(group, hebi_feedback, command, theta1i, theta2i, type):
    print(' ')
    print('Moving to initial position...')
    theta_d = np.array([theta1i, theta2i])
    omega_d = np.zeros(2)
    converged1 = False
    converged2 = False
    converged3 = False

    slack = 0
    if type == "encoder":
        slack = 10*(np.pi/180)

    t, t0 = reset_timer()
    while not converged2:
        h_theta, h_omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback) 
        t = loop_timer(t0, 0.01, print_loop_time=False)
        if not converged1:
            effort, converged1 = initializing_controller1(h_theta,theta_d,h_omega,omega_d, init_tol1, slack)
        else:
            effort, converged2 = initializing_controller2(h_theta,theta_d,init_tol2, slack)
        command.effort = effort
        send_hebi_effort_command(group, command)

    if type == "encoder":
        sleep(0.2)
        print("Resetting backlash...")
        while not converged3:
            h_theta, h_omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback) 
            t = loop_timer(t0, 0.01, print_loop_time=False)
            effort, converged3 = initializing_controller3(h_theta,theta_d,h_omega,omega_d, init_tol3)
            command.effort = effort
            send_hebi_effort_command(group, command)

    command.effort = np.nan
    command.position = np.nan
    sleep(0.5)
    print('Initialization complete')
    print(' ')
    print('Running Trajectory...')
    print(' ')
    return


# ===========================================================================================================

if __name__ == "__main__":
    output = []
    freq = 100 # hz
    
    group, hebi_feedback, command = initialize_hebi()
    group.feedback_frequency = freq
    group_info = group.request_info()

    if group_info is not None:
        group_info.write_gains("csv/saved_gains.xml")

    theta1i = 0.0227
    theta2i = 1.1095

    set_hebi_position(group, hebi_feedback, command, theta1i, theta2i, type)

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





