from controller_functions import *
from trajectory_functions import Trajectory
from numpy import pi, sin, cos
from os import path
import matplotlib.pyplot as plt



#=== Change these to gather trials ===#
preview_time = 1
oneD = False
trajectory_type = "sines"
trial = 1
f = 0.025 # default: 0.025
T = 1/f
T = 120

bl_switch_freq = 2 # hz
threshold_bl = 0.0
user_cutoff_freq = 3.0

#=== Global variables ===#

# Link lengths for 2 DOF:
L1 = 0.285
L2 = 0.265

# Initialize user input filter:
O1k_1 = 0 # y1[k-1]
V1k_1 = 0 # v1[k-1]
O2k_1 = 0 # y2[k-1]
V2k_1 = 0 # v2[k-1]

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

def save_data(output):
    fs = str(f)
    f_r = fs.replace('.', '')
    save_name = "csv/{}_{}_{}.csv".format(trajectory_type, f_r, trial)
    #for i in range(2, 100):
    #    if path.exists(save_name) is True:
    #        save_name = "csv/{}_{}_{}_{}.csv".format(type, f_r, comp, i)
    #    else:
    #        break

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

       return omega_d, vel_ommitd

if __name__ == "__main__":

    print("T:", T)
    joystick = initialize_joystick()
    output = []

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)
    trajectory = Trajectory(trajectory_type, T, f, window_size, trial)

    t_max, vel_max = trajectory.max_vel()

    pos_i = trajectory.screen_coordinates(0)
    target_ball = Ball(pos_i, target_ball_radius, "red", animation_canvas)

    line = animation_canvas.create_line(0, 0, 0, 0, fill='red', smooth='true', dash=(6,4))
    # line = animation_canvas.create_line(0, 0, 0, 0, fill='red', arrow='last', smooth='true', dash=(6,4))

    gain = vel_max
    input_ball = Ball(pos_i, input_ball_radius, "white", animation_canvas)
    pos_input = pos_i
    
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

    while True:
        count += 1

        t = loop_timer(t0w, Tw, print_loop_time=True)

        pos_input, t_draw = controller_draw(joystick,pos_input,t_draw,gain)
        pos_input[1] = 450
           
        theta_out = np.zeros(2)

        draw_preview(animation_canvas, line, trajectory, preview_time, T, t)
        pos = target_ball.move(trajectory.screen_coordinates(t))
        target_ball.move(pos)

        input_ball.move(pos_input)
        animation_window.update()
        error = np.sqrt(np.sum(np.square(pos-pos_input)))
        rmse_sum += np.sum(np.square(pos-pos_input))

        output += [[t, pos_input[0], pos_input[1], pos[0], pos[1], error]]

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
            plt.plot(output[:,0], output[:,5])
            plt.xlabel('Time [s]')
            plt.ylabel('Position Error [pixels]')
            plt.title('Error (Total RMSE='+str(round(rmse))+' Pixels)')
            plt.legend()
         
            plt.show()

            break