from hebi_position import calculate_hebi_position, set_hebi_position, reset_timer, loop_timer
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command
from encoder_position import calculate_encoder_position
from encoder_functions import initialize_encoders
from controller_functions import *
from numpy import pi, sin, cos
from os import path
from backlash_functions import smooth_backlash_inverse
from friction_functions import smooth_fric_comp
import matplotlib.pyplot as plt


#=== Change these to gather trials ===#
#type = "hebi"
#type = "controller"
type = "encoder"
backlash_compensation = False
friction_compensation = False
f = 0.025
T = 1/f


#=== Global variables ===#

# Link lengths for 2 DOF:
L1 = 0.27
L2 = 0.25

# Initialize inner PI controller integral error sum:
inner_PI_e1 = 0
inner_PI_e2 = 0

# Initialize user input filter:
y1k_1 = 0 # y1[k-1]
v1k_1 = 0 # v1[k-1]
y2k_1 = 0 # y2[k-1]
v2k_1 = 0 # v2[k-1]

def PI_controller1(theta,theta_d, Kpi):    
    global inner_PI_e1
    kp1 = Kpi[0]
    ki1 = Kpi[1]
    inner_PI_e1 += theta_d-theta
    return kp1*(theta_d-theta) + ki1*inner_PI_e1

def PI_controller2(theta,theta_d, Kpi):    
    global inner_PI_e2
    kp2 = Kpi[2]
    ki2 = Kpi[3]
    inner_PI_e2 += theta_d-theta
    return kp2*(theta_d-theta) + ki2*inner_PI_e2

def user_input_filter(omega, cutoff_freq=1.0, T=0.01):
    # input freq in hz
    global y1k_1
    global v1k_1
    global y2k_1
    global v2k_1
    
    tau = 1/(2*pi*cutoff_freq)
    y1 = (omega[0] + v1k_1 - (1-2*tau/T)*y1k_1)/(2*tau/T+1)
    y2 = (omega[1] + v2k_1 - (1-2*tau/T)*y2k_1)/(2*tau/T+1)
    y1k_1 = y1 
    v1k_1 = omega[0]
    y2k_1 = y2
    v2k_1 = omega[1]
    return np.array([y1,y2])

def save_data(output):
    fs = str(f)
    f_r = fs.replace('.', '')
    save_name = "csv/{}_{}_1.csv".format(type, f_r)
    for i in range(2, 100):
        if path.exists(save_name) is True:
            save_name = "csv/{}_{}_{}.csv".format(type, f_r, i)
        else:
            break

    np.savetxt(save_name, np.array(output), delimiter=",")
    print("Data saved as:", save_name)

def calculate_velocity(theta, joystick, K):
       axis = get_axis(joystick)
       axis[1] = -axis[1]
       theta1 = theta[0]
       theta2 = theta[1]

       Jinv = np.matrix([[-cos(theta1 + theta2)/(L1*cos(theta2)), -sin(theta1 + theta2)/(L1*cos(theta2))],
                         [(L2*cos(theta1 + theta2) + L1*sin(theta1))/(L1*L2*cos(theta2)), (L2*sin(theta1 + theta2) - L1*cos(theta1))/(L1*L2*cos(theta2))]])
       # print("Jinv:", Jinv)
       omega_d = Jinv @ K @ axis
       omega_d = np.squeeze(np.asarray(omega_d))

       return omega_d

if __name__ == "__main__":

    joystick = initialize_joystick()
    t_max, vel_max = max_vel(f)
    output = []

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)


    if type == "hebi" or type == "encoder":
        K = 0.3*(1/window_size)*vel_max*np.matrix([[1, 0],
                                                   [0, 1]]) # gain was 0.6
        #K = np.matrix([[0.02, 0],
        #               [0, 0.02]])
        print("Gain matrix:", K)
        freq = 100 # hz
        group, hebi_feedback, command = initialize_hebi()
        group.feedback_frequency = freq
        theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)
        group_info = group.request_info()
        if group_info is not None:
            group_info.write_gains("csv/saved_gains.xml")
        
        theta1i = 0.4599
        theta2i = 0.4156
        set_hebi_position(group, hebi_feedback, command, theta1i, theta2i)

    pos_i = screen_trajectory(0, f)
    target_ball = Ball(pos_i, target_ball_radius, "red", animation_canvas)

    if type == "controller":
        gain = vel_max
        print(gain)
        input_ball = Ball(pos_i, input_ball_radius, "white", animation_canvas)
        pos_input = pos_i

    if type == "hebi":
        pos0 = calculate_hebi_position(group, hebi_feedback, offset = 0)
        offset = pos_i - pos0
        input_ball = Ball(calculate_hebi_position(group, hebi_feedback, offset), input_ball_radius, "white", animation_canvas)
        print("Offset 1:", offset)
        # print(calculate_hebi_position(group, hebi_feedback, offset))

    if type == "encoder":
        arduino = initialize_encoders()
        pos0 = calculate_encoder_position(arduino, offset = 0)
        offset = pos_i - pos0
        input_ball = Ball(calculate_encoder_position(arduino, offset), input_ball_radius, "white", animation_canvas)
    
    animation_window.update()

    print("Get ready...")
    sleep(1)

    i = 0
    t0 = time()
    t_draw = t0

    _, t0w = reset_timer()
    Tw = 0.01
    theta_v = np.array([0.0, 0.0])

    omega_d_1 = np.zeros(2) # omega_d[k-1]
    omega_f_1 = np.zeros(2) # omega_d[k-1]
    theta_d = np.nan*np.ones(2)
    user_cutoff_freq = 1.0
    bl_cutoff_freq = 0.1
    v_d = np.nan*np.ones(2) # velocity command to hebi
    e_d = np.nan*np.ones(2) # effort command to hebi
    e_fric = np.zeros(2)
    backlash_converged0 = True
    backlash_converged1 = True
    Kpi = [10, 0.0, 10, 0.0] # [Kp1, Ki1, Kp2, Ki2]
    converged_tol = 0.001
    c_R = [-1.62e-2, 6.2e-2]
    c_L = [1.58e-1, 3.13e-1]
    m = [9.67e-1, 9.78e-1]
    fsp = [1.21,7.45e-1]
    fsn = [-7.04e-1,-1.53]
    B0 = [2.2,3.42]

    human_theta_d, _, _, _ = get_hebi_feedback(group, hebi_feedback)  

    while True:

       t = time() - t0
       _ = loop_timer(t0w, Tw, print_loop_time=False)

       #print("Effort:", command.effort)
       # print("Velocity", command.velocity)
       #print("Position", command.position)
       if type == "controller":
           pos_input, t_draw = controller_draw(joystick,pos_input,t_draw,gain)
           
       if type == "hebi":
           pos_input = calculate_hebi_position(group, hebi_feedback, offset)

       if type == "encoder":
           pos_input = calculate_encoder_position(arduino, offset)

       if type == "hebi" or type == "encoder":
            theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)  
            #print("Theta:", theta)
            omega_d = calculate_velocity(theta, joystick, K)
            command.velocity = omega_d
            if not backlash_compensation:
                if friction_compensation:
                    command.effort = smooth_fric_comp(theta, omega_d, fsp=fsp, fsn=fsn, B0=B0)
                group.send_command(command)
        
            else:
                omega_f = user_input_filter(omega_d, cutoff_freq=user_cutoff_freq, T=Tw)
                human_theta_d += omega_d * Tw
                theta_d, backlash_converged0, backlash_converged1 = smooth_backlash_inverse(human_theta_d, omega_f, c_R=c_R, c_L=c_L, m=m, cutoff_freq=bl_cutoff_freq, converged_tol=converged_tol)

                if (np.sign(omega_f[0]) != np.sign(omega_f_1[0])):
                    human_theta_d[0] = theta[0]
                if (np.sign(omega_f[1]) != np.sign(omega_f_1[1])):
                    human_theta_d[1] = theta[1]

                if backlash_converged0:
                    e_d[0] = np.nan
                else:
                    e_d[0] = PI_controller1(theta[0], theta_d[0], Kpi)
                if backlash_converged1:
                    e_d[1] = np.nan
                else:
                    e_d[1] = PI_controller2(theta[1], theta_d[1], Kpi)                    

                omega_f_1 = omega_f

                if friction_compensation:
                    e_fric = smooth_fric_comp(theta, omega_d, fsp=fsp, fsn=fsn, B0=B0)

                command.effort = e_d + e_fric
                #command.effort = [np.nan, np.nan]
                group.send_command(command)  


       pos = screen_trajectory(t, f)
       target_ball.move(pos)
       input_ball.move(pos_input)
       animation_window.update()

       output += [[t, omega[0], omega[1], omega_d[0], omega_d[1]]]

       if i == 0:
           print("Ready to operate...")
           i = 1

       if t > T:
           save_data(output)
           output = np.array(output)
           print("Trajectory complete, saving data")
           plt.plot(output[:, 0], output[:, 1], label = "Feedback Velocity 1")
           plt.plot(output[:, 0], output[:, 2], label = "Feedback Velocity 2")
           plt.plot(output[:, 0], output[:, 3], label = "Desired Velocity 1")
           plt.plot(output[:, 0], output[:, 4], label = "Desired Velocity 2")
           plt.legend()
           plt.show()
           break

       if keyboard.is_pressed('esc'):
           print("Trajectory interupted")
           break
