from hebi_position import calculate_hebi_position, set_hebi_position, reset_timer, loop_timer
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command, get_hebi_feedback_without_random_offsets
from encoder_position import calculate_encoder_position
from encoder_functions import initialize_encoders, get_encoder_feedback_without_random_offsets
from controller_functions import *
from trajectory_functions import Trajectory
from numpy import pi, sin, cos
from os import path
from backlash_functions import inverse_hammerstein
import matplotlib.pyplot as plt

# Initialize inner PI controller integral error sum:
inner_PI_e1 = 0
inner_PI_e2 = 0

#=== Change these to gather trials ===#
preview_time = 0
oneD = False
# type = "hebi"
#type = "controller"
type = "encoder"
trajectory_type = "sineInterp"
trial = 1
backlash_compensation = True
include_GPR = False
model_number = '1'
f = 0.025 # default: 0.025
T = 1/f

bl_switch_freq = 2 # hz
threshold_bl = 0.0
user_cutoff_freq = 3.0

Kp = [45.0, 40.0] # [Kp1, Kp2], [50.0, 40.0]

# hammerstein inverse parameters
c_bl = [-0.0147, -0.1292, -0.0179, -0.2218]
K_bl = [0.9703, 0.9843]
tau_bl = [0.1, 0.1, 0.26, 0.38]

# smooth inverse parameters
#bl_cutoff_freq = 0.5
#c_R = [0.02275, 0.043848]
#c_L = [0.18483, 0.314722]
#m = [0.97676, 0.97014]


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

def inner_PI_controller(theta,theta_d):    
    global inner_PI_e1
    global inner_PI_e2
    
    # Gains with cables:
    kp1 = 35.0 # 60 # 45
    ki1 = 2.0 # 0.8 # 3.0
    kp2 = 50.0 # 60.0 
    ki2 = 2.0 # 4.0
    
    inner_PI_e1 += theta_d[0]-theta[0]
    inner_PI_e2 += theta_d[1]-theta[1]
    PI_effort = np.array([kp1*(theta_d[0]-theta[0]) + ki1*inner_PI_e1,
                          kp2*(theta_d[1]-theta[1]) + ki2*inner_PI_e2])
    return PI_effort

def save_data(output):
    if backlash_compensation:
        comp = "compensated"
    else:
        comp = "uncompensated"
    fs = str(f)
    f_r = fs.replace('.', '')
    save_name = "csv/{}_{}_{}_1.csv".format(type, f_r, comp)
    for i in range(2, 100):
        if path.exists(save_name) is True:
            save_name = "csv/{}_{}_{}_{}.csv".format(type, f_r, comp, i)
        else:
            break

    np.savetxt(save_name, np.array(output), delimiter=",")
    print("Data saved as:", save_name)

def calculate_velocity(theta, joystick, K):
        
       axis = get_axis(joystick)
       axis[1] = -axis[1]

       pos = input_ball.pos
       if pos[0] < input_ball_radius:
           axis[0] = 0.75
       elif pos[0] > window_size - input_ball_radius:
           axis[0] = -0.75
       elif pos[1] < input_ball_radius:
           axis[1] = -0.75
       elif pos[1] > window_size - input_ball_radius:
           axis[1] = 0.75
       theta1 = theta[0]
       theta2 = theta[1]

       Jinv = np.matrix([[-cos(theta1 + theta2)/(L1*cos(theta2)), -sin(theta1 + theta2)/(L1*cos(theta2))],
                         [(L2*cos(theta1 + theta2) + L1*sin(theta1))/(L1*L2*cos(theta2)), (L2*sin(theta1 + theta2) - L1*cos(theta1))/(L1*L2*cos(theta2))]])
       # print("Jinv:", Jinv)
       vel_ommitd = K @ axis
       omega_d = Jinv @ K @ axis
       omega_d = np.squeeze(np.asarray(omega_d))

       return omega_d, vel_d

if __name__ == "__main__":

    print("T:", T)
    joystick = initialize_joystick()
    output = []

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)
    trajectory = Trajectory(trajectory_type, T, f, window_size, trial)
    if trajectory_type == "sineInterp":
        trajectory.trialLookup()

    print("T in class:", trajectory.T)
    t_max, vel_max = trajectory.max_vel()

    if backlash_compensation:
        if include_GPR:
            GPR_models = load_GPR_param_models(model_number)
        else:
            GPR_models = None


    if type == "hebi" or type == "encoder":

        K_gain = 1.5
        K_gain = 0.2
        workspace_size = 0.37
        K = K_gain*(workspace_size/window_size)*vel_max*np.matrix([[1, 0], [0, 1]])
        print("Gain matrix:", K)
        freq = 100 # hz
        group, hebi_feedback, command = initialize_hebi()
        group.feedback_frequency = freq
        theta, omega, torque = get_hebi_feedback(group, hebi_feedback)
        group_info = group.request_info()
        if group_info is not None:
            group_info.write_gains("csv/saved_gains.xml")
        if trajectory_type == "circle":
            theta1i = 0.2827
            theta2i = 1.0694

        if trajectory_type == "chirp" or trajectory_type == "chirp2" or trajectory_type == "sines":
            theta1i = 1.3728
            theta2i = 0.8586

        if trajectory_type == "sineInterp":
            theta1i = self.thetai1
            theta2i = self.thetai2


        set_hebi_position(group, hebi_feedback, command, theta1i, theta2i, type)

    pos_i = trajectory.screen_coordinates(0)
    target_ball = Ball(pos_i, target_ball_radius, "red", animation_canvas)

    line = animation_canvas.create_line(0, 0, 0, 0, fill='red', arrow='last', smooth='true', dash=(6,4))

    if type == "controller":
        gain = 2*vel_max
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
    
    draw_preview(animation_canvas, line, trajectory, preview_time, T, 0)
    animation_window.update()

    print("Get ready...")
    sleep(1)

    i = 0
    t0 = time()
    t_draw = t0

    # set timer
    _, t0w = reset_timer()
    Tw = 0.01

    # collect error metrics
    count = 0
    rmse_sum = 0

    if type != "controller":     # this was only for encoder w/ backlash but I think it should be for everything except controller
        theta_init, _, _ = get_hebi_feedback_without_random_offsets(group, hebi_feedback)  
        user_theta_d = np.array(theta_init) #- np.array([c_bl[1], c_bl[3]]) # initialize 
        theta_init, _, _ = get_hebi_feedback_without_random_offsets(group, hebi_feedback)  
        user_theta_d = np.array(theta_init) #- np.array([c_bl[1], c_bl[3]]) # initialize backlash

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
            trajectory.coordinates(t)
            K = K_gain * trajectory.K * np.eye(2)
            theta, omega, torque = get_hebi_feedback(group, hebi_feedback)  
            omega_d, vel_d = calculate_velocity(theta, joystick, K)

            if type == "encoder" and backlash_compensation:
                user_theta_d += omega_d * Tw
                omega_f = user_input_filter(omega_d, cutoff_freq=user_cutoff_freq, T=Tw)
                theta_d = inverse_hammerstein(user_theta_d, omega_d,omega_f, GPR_models=GPR_models, c=c_bl, K=K_bl, tau=tau_bl, threshold=threshold_bl, switch_freq=bl_switch_freq, T=Tw)
                theta_controller, _, _ = get_hebi_feedback_without_random_offsets(group, hebi_feedback) 
                #e_d = P_controller(theta_controller, theta_d, Kp)
                e_d = inner_PI_controller(theta_controller, theta_d)
                command.effort = e_d
                group.send_command(command)
            else:
                user_theta_d += omega_d * Tw
                theta_controller, _, _ = get_hebi_feedback_without_random_offsets(group, hebi_feedback)
                e_d = inner_PI_controller(theta_controller, user_theta_d)
                command.effort = e_d
                group.send_command(command)
                omega_f = omega_d
                theta_d = user_theta_d
                #command.velocity = omega_d
                #group.send_command(command)

       if type == "encoder":
            theta_out = get_encoder_feedback_without_random_offsets(arduino)
       else: theta_out = np.zeros(2)

       draw_preview(animation_canvas, line, trajectory, preview_time, T, t)
       pos = target_ball.move(trajectory.screen_coordinates(t))
       target_ball.move(pos)

       input_ball.move(pos_input)
       animation_window.update()
       error = np.sqrt(np.sum(np.square(pos-pos_input)))
       rmse_sum += np.sum(np.square(pos-pos_input))

       if type == "encoder" and backlash_compensation:
           output += [[t, pos_input[0]*0.00037, pos_input[1]*0.00037, pos[0]*0.00037, pos[1]*0.00037,
                      vel_d[0], vel_d[1], error, omega_d[0], omega_d[1], omega_f[0], omega_f[1], omega[0], omega[1],
                      theta_controller[0], theta_controller[1], theta_d[0], theta_d[1], theta_out[0], theta_out[1]]]
       elif type == "controller":
           output += [[t, pos_input[0], pos_input[1], pos[0], pos[1], error]]
       else:
           #output += [[t, pos_input[0], pos_input[1], pos[0], pos[1], error]]
           #output += [[t, pos_input[0]*0.00037, pos_input[1]*0.00037, pos[0]*0.00037, pos[1]*0.00037, error]]
           output += [[t, pos_input[0]*0.00037, pos_input[1]*0.00037, pos[0]*0.00037, pos[1]*0.00037,
                       vel_d[0], vel_d[1], error, omega_d[0], omega_d[1], omega_f[0], omega_f[1], omega[0], omega[1],
                       theta_controller[0], theta_controller[1], theta_d[0], theta_d[1], theta_out[0], theta_out[1]]]

       if i == 0:
           print("Ready to operate...")
           i = 1

       if t > T or keyboard.is_pressed('esc'):
           rmse = np.sqrt(rmse_sum/count)
           save_data(output)
           output = np.array(output)
           print("Trajectory complete, data saved")

           velm1 = np.diff(output[:,1]*0.00037)/np.diff(output[:,0])
           velm2 = np.diff(output[:,2]*0.00037)/np.diff(output[:,0])

           velc1 = np.diff(output[:,3]*0.00037)/np.diff(output[:,0])
           velc2 = np.diff(output[:,4]*0.00037)/np.diff(output[:,0])

           if type == "encoder" and backlash_compensation:
               plt.figure()
               plt.plot(output[:,0], output[:,8], label = "Omega desired 1")
               plt.plot(output[:,0], output[:,9], label = "Omega desired 2")
               plt.plot(output[:,0], output[:,10], label = "Omega desired 1 filtered")
               plt.plot(output[:,0], output[:,11], label = "Omega desired 2 filtered")
               plt.plot(output[:,0], output[:,12], label = "Omega feedback 1")
               plt.plot(output[:,0], output[:,13], label = "Omega feedback 2")
               plt.legend()
               plt.xlabel('Time [s]')
               plt.ylabel('Velocity [rad/s]')
               plt.title('User Input Desired Angular Velocities')

           
               plt.figure()
               plt.plot(output[:,0], output[:,14], label = "Actual Theta 1")
               plt.plot(output[:,0], output[:,15], label = "Actual Theta 2")
               plt.plot(output[:,0], output[:,16], label = "Desired Theta 1")
               plt.plot(output[:,0], output[:,17], label = "Desired Theta 2")
               plt.legend()
               plt.xlabel('Time [s]')
               plt.ylabel('Velocity [rad/s]')
               plt.title('User Input Desired Angular Velocities')

           plt.figure()
           plt.plot(output[:,0], output[:,1], label = "Actual Position x")
           plt.plot(output[:,0], output[:,2], label = "Actual Position y")
           plt.plot(output[:,0], output[:,3], label = "Desired Position x")
           plt.plot(output[:,0], output[:,4], label = "Desired Position y")
           plt.legend()
           plt.xlabel('Time [s]')
           plt.ylabel('Position [pixels]')
           plt.title('Position')

           plt.figure()
           if type == "encoder" and backlash_compensation:
               plt.plot(output[:,0], output[:,7])
           else:
               plt.plot(output[:,0], output[:,5])
           plt.xlabel('Time [s]')
           plt.ylabel('Position Error [pixels]')
           plt.title('Error (Total RMSE='+str(round(rmse))+' Pixels)')
           plt.legend()
           if type == "encoder" and backlash_compensation:
               plt.figure()
               plt.plot(output[1:,0], velc1, 'k--', linewidth=1.0, label="Desired Velocity x")
               plt.plot(output[1:,0], -velc2, 'm--', linewidth=1.0, label="Desired Velocity y")
               plt.plot(output[:,0], output[:,5], 'b--', linewidth=1.5, label="User Desired Velocity x")
               plt.plot(output[:,0], output[:,6], 'r--', linewidth=1.5, label="User Desired Velocity y")
               plt.plot(output[1:,0], velm1, 'b', linewidth=1.5, label="Motor Velocity x")
               plt.plot(output[1:,0], -velm2, 'r', linewidth=1.5, label="Motor Velocity y")
               plt.xlabel('Time [s]')
               plt.ylabel('Position')
               plt.title('User Desired vs Actual Velocity')
               plt.legend()

           plt.show()

           break

       #if keyboard.is_pressed('esc'):
       #    save_data(output)
       #    print("Trajectory interupted")
       #    #break
       #    while True and type is not "controller":
       #        command.velocity = np.nan*np.ones(2)
       #        command.effort = np.zeros(2)
       #        group.send_command(command)
       #    else:
       #        break

