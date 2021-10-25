from hebi_position import calculate_hebi_position, set_hebi_position, reset_timer, loop_timer
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command
from encoder_position import calculate_encoder_position
from encoder_functions import initialize_encoders
from controller_functions import *
from numpy import pi, sin, cos
from os import path
from backlash_functions import smooth_backlash_inverse, load_GPR_param_models
import matplotlib.pyplot as plt


#=== Change these to gather trials ===#
#type = "hebi"
#type = "controller"
type = "encoder"
backlash_compensation = 1
include_GPR = 0
model_number = '1'
f = 0.025 # default: 0.025
T = 1/f


#=== Global variables ===#

# Link lengths for 2 DOF:
L1 = 0.285
L2 = 0.265

# Initialize user input filter:
O1k_1 = 0 # y1[k-1]
V1k_1 = 0 # v1[k-1]
O2k_1 = 0 # y2[k-1]
V2k_1 = 0 # v2[k-1]

def P_controller(theta,theta_d,Kp):    
    return np.array([Kp[0]*(theta_d[0]-theta[0]), Kp[1]*(theta_d[1]-theta[1])])

def user_input_filter(omega, cutoff_freq=1.0, T=0.01):
    # input freq in hz
    global O1k_1
    global V1k_1
    global O2k_1
    global V2k_1
    tau = 1/(2*pi*cutoff_freq)
    O1 = (omega[0] + V1k_1 - (1-2*tau/T)*O1k_1)/(2*tau/T+1)
    O2 = (omega[1] + V2k_1 - (1-2*tau/T)*O2k_1)/(2*tau/T+1)
    O1k_1 = O1 
    V1k_1 = omega[0]
    O2k_1 = O2
    V2k_1 = omega[1]
    return np.array([O1,O2])

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

    if backlash_compensation:
        if include_GPR:
            GPR_models = load_GPR_param_models(model_number)
        else:
            GPR_models = None


    if type == "hebi" or type == "encoder":

        # gains:
        #   0.23 backlash comp joystick f=0.025
        #   0.6 no backlash comp joystick f=0.025

        if backlash_compensation: 
            K_gain = 0.45
        else: 
            K_gain = 0.50
        K = K_gain*(1/window_size)*vel_max*np.matrix([[1, 0],
                                                      [0, 1]]) # gain was 0.3 with joystick
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
    user_cutoff_freq = 1.5
    bl_cutoff_freq = 0.5
    Kp = [50.0, 40.0] # [Kp1, Kp2]
    c_R = [-0.033, 0.027]
    c_L = [0.1598, 0.5273]
    m = [0.9627, 0.9675]
    count = 0
    rmse_sum = 0

    if type == "encoder" or type == "hebi":
        human_theta_d, _, _, _ = get_hebi_feedback(group, hebi_feedback)  

    while True:
       count += 1

       t = loop_timer(t0w, Tw, print_loop_time=False)

       if type == "controller":
           pos_input, t_draw = controller_draw(joystick,pos_input,t_draw,gain)
           
       if type == "hebi":
           pos_input = calculate_hebi_position(group, hebi_feedback, offset)

       if type == "encoder":
           pos_input = calculate_encoder_position(arduino, offset)

       if type == "hebi" or type == "encoder":
            theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)  
            omega_d = calculate_velocity(theta, joystick, K)
            
            if not backlash_compensation:
                command.velocity = omega_d
                group.send_command(command)
            else:
                omega_f = user_input_filter(omega_d, cutoff_freq=user_cutoff_freq, T=Tw)
                human_theta_d += omega_f * Tw
                theta_d = smooth_backlash_inverse(human_theta_d, omega_f, GPR_models=GPR_models, c_R=c_R, c_L=c_L, m=m, cutoff_freq=bl_cutoff_freq)   
                e_d = P_controller(theta, theta_d, Kp)
                command.effort = e_d
                group.send_command(command)

       pos = screen_trajectory(t, f)
       target_ball.move(pos)
       input_ball.move(pos_input)
       animation_window.update()
       error = np.sqrt(np.sum(np.square(pos-pos_input)))
       rmse_sum += np.sum(np.square(pos-pos_input))
       output += [[t, pos_input[0], pos_input[1], pos[0], pos[1], error]]

       if i == 0:
           print("Ready to operate...")
           i = 1

       if t > T:
           rmse = np.sqrt(rmse_sum/count)
           save_data(output)
           output = np.array(output)
           print("Trajectory complete, saving data")
           plt.figure()
           plt.plot(output[:, 0], output[:, 1], label = "Actual Position x")
           plt.plot(output[:, 0], output[:, 2], label = "Actual Position y")
           plt.plot(output[:, 0], output[:, 3], label = "Desired Position x")
           plt.plot(output[:, 0], output[:, 4], label = "Desired Position y")
           plt.legend()
           plt.xlabel('Time [s]')
           plt.ylabel('Position [pixels]')
           plt.title('Position')
           plt.figure()
           plt.plot(output[:, 0], output[:, 5])
           plt.xlabel('Time [s]')
           plt.ylabel('Position Error [pixels]')
           plt.title('Error (Total RMSE='+str(round(rmse))+' Pixels)')
           plt.legend()
           plt.show()
           break

       if keyboard.is_pressed('esc'):
           print("Trajectory interupted")
           break

