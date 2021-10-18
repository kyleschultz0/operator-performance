from hebi_position import calculate_hebi_position
from hebi_functions import initialize_hebi, get_hebi_feedback, send_hebi_position_command, send_hebi_effort_command
from encoder_position import calculate_encoder_position
from encoder_functions import initialize_encoders
from controller_functions import *
from numpy import pi, sin, cos

type = "encoder"
f = 0.05

#=== Global variables ===#

# Initialize input filter:
y1k_1 = 0 # y1[k-1]
f1k_1 = 0 # f1[k-1]
y2k_1 = 0 # y2[k-1]
f2k_1 = 0 # f2[k-1]



def input_filter(input, cutoff_freq, T):
    # input freq in hz
    global y1k_1
    global f1k_1
    global y2k_1
    global f2k_1
    
    tau = 1/(2*np.pi*cutoff_freq)
    y1 = (input[0] + f1k_1 - (1-2*tau/T)*y1k_1)/(2*tau/T+1)
    y2 = (input[1] + f2k_1 - (1-2*tau/T)*y2k_1)/(2*tau/T+1)
    y1k_1 = y1 
    f1k_1 = input[0]
    y2k_1 = y2
    f2k_1 = input[1]
    return np.array([y1,y2])

def save_data(output):
    fs = str(f)
    f_r = fs.replace('.', '')
    save_name = "csv/{}_{}_1".format(type, f_r)
    if path.exists(save_name) is True:
        save_name = "csv/{}_{}_2".format(type, f_r)

    np.savetxt(save_name, np.array(output), delimiter=",")
    print("Data saved as:", save_name)

def calculate_velocity(theta, joystick, K):
       axis = get_axis(joystick)
       axis[1] = -axis[1]
       theta1 = theta[0]
       theta2 = theta[1]

       Jinv = np.matrix([[-cos(theta1 + theta2)/(L1*cos(theta2)), -sin(theta1 + theta2)/(L1*cos(theta2))],
                         [(L2*cos(theta1 + theta2) + L1*sin(theta1))/(L1*L2*cos(theta2)), (L2*sin(theta1 + theta2) - L1*cos(theta1))/(L1*L2*cos(theta2))]])

       omega_d = Jinv @ K @ axis
       omega_d = np.squeeze(np.asarray(omega_d))

       return omega_d


if __name__ == "__main__":

    joystick = initialize_joystick()
    output = []
    t_max, vel_max = max_vel(f)

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)

    pos_i = screen_trajectory(0, f)
    target_ball = Ball(pos_i, target_ball_radius, "red", animation_canvas)

    if type == "controller":
        gain = 500*vel_max
        print(gain)
        input_ball = Ball(pos_i, input_ball_radius, "white", animation_canvas)
        pos_input = pos_i

    if type == "hebi":
        K = 0.1*vel_max*np.matrix([[1, 0],
                                   [0, 1]])
        freq = 100 # hz
        group, hebi_feedback, command = initialize_hebi()
        group.feedback_frequency = freq
        theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)
        group_info = group.request_info()
        pos0 = calculate_hebi_position(group, hebi_feedback, offset = 0)
        offset = pos_i - pos0
        input_ball = Ball(calculate_hebi_position(group, hebi_feedback, offset), input_ball_radius, "white", animation_canvas)
        print(calculate_hebi_position(group, hebi_feedback, offset))
        if group_info is not None:
            group_info.write_gains("csv/saved_gains.xml")


    if type == "encoder":
        arduino = initialize_encoders()
        pos0 = calculate_encoder_position(arduino, offset = 0)
        offset = pos_i - pos0
        input_ball = Ball(calculate_encoder_position(arduino, offset), input_ball_radius, "white", animation_canvas)
    
    animation_window.update()

    print("Get ready...")
    sleep(1)



    #=== variables for 2 dof ===#
    L1 = 0.29
    L2 = 0.22
    #======#

    #=== variables for 3 dof ===#
    # l1 = 0.268
    # l2 = 0.472
    #======#


    i = 0
    t0 = time()
    t_draw = t0

    while True:
       t = time() - t0

       if type == "controller":
           pos_input, t_draw = controller_draw(joystick,pos_input,t_draw,gain)

       if type == "hebi" or type == "encoder":
           theta, omega, torque, hebi_limit_stop_flag = get_hebi_feedback(group, hebi_feedback)  
           theta = theta - np.array([1.58702857, -0.08002613])
           omega_d = calculate_velocity(theta, joystick, K)
           command.velocity = omega_d
           group.send_command(command)
           
       if type == "hebi":
           pos_input = calculate_hebi_position(group, hebi_feedback, offset)

       if type == "encoder":
           pos_input = calculate_encoder_position(arduino, offset)


       pos = screen_trajectory(t, f)
       target_ball.move(pos)
       input_ball.move(pos_input)
       animation_window.update()

       if i == 0:
           print("ready to operate...")
           i = 1


       if keyboard.is_pressed('esc'):
           # save_data(output)
           break
