import pickle
import numpy as np

### global variables for filters ###
global u_s
u_s = np.zeros(2)
p1k_1 = 0 # p1[k-1] - output
q1k_1 = 0 # q1[k-1] - input
p2k_1 = 0 # p2[k-1] - output
q2k_1 = 0 # q2[k-1] - input

### list of functions ###
# load_GPR_param_models(model_number)
    # loads GPR parameter models for joint 2
# init_u_s_filter(theta)
    # initialize filter to avoid extreme initial jumps
# u_s_filter(q, cutoff_freq, T=0.01)
    # filter for smooth inverse switching
# def smooth_backlash_inverse(theta, omega, GPR_models=None, cutoff_freq=0.85, c_R=[0,0], c_L=[0,0], m=[1,1])
    # call in loop for command to HEBI controller

def load_GPR_param_models(model_number):
    GPR_models = []
    model_names = ['c_R', 'c_L', 'm', 'fsp', 'fsn', 'B']
    for model_name in model_names:
        pkl_filename = 'GPR' + model_number + '_' + model_name + '.pkl'
        with open(pkl_filename, 'rb') as file:
            GPR_models += [pickle.load(file)]
    print('GPR models loaded')
    return GPR_models

def initialize_backlash(c_init, m, theta_init):
    global u_s
    u_s = -c_init

    theta_init[0] *= m[0]
    theta_init[0] -= u_s[0]
    theta_init[1] *= m[1]
    theta_init[1] -= u_s[1]

    return theta_init

def u_s_filter(q, cutoff_freq, T=0.01):
    global p1k_1
    global q1k_1
    global p2k_1
    global q2k_1

    tau = 1/(2*np.pi*cutoff_freq) 
    p1 = (q[0] + q1k_1 - (1-2*tau/T)*p1k_1)/(2*tau/T+1)
    p2 = (q[1] + q2k_1 - (1-2*tau/T)*p2k_1)/(2*tau/T+1)
    p1k_1 = p1 
    q1k_1 = q[0]
    p2k_1 = p2 
    q2k_1 = q[1]
    return np.array([p1,p2])

def smooth_backlash_inverse(theta, omega, GPR_models=None,
                            cutoff_freq=0.85, T=0.01,
                            c_R=[0,0], c_L=[0,0], m=[1,1]):
    global u_s

    if GPR_models != None:
        c_R[1] = GPR_models[0].predict(theta[0].reshape(1,-1), return_std=False)[0]
        c_L[1] = GPR_models[1].predict(theta[0].reshape(1,-1), return_std=False)[0] 
        m[1] = GPR_models[2].predict(theta[0].reshape(1,-1), return_std=False)[0]
    if omega[0] > 0:
        u_s[0] = -c_R[0]
    elif omega[0] < 0:
        u_s[0] = -c_L[0]
    else: pass # u_s[k] = u_s[k-1]
    if omega[1] > 0:
        u_s[1] = -c_R[1]
    elif omega[1] < 0:
        u_s[1] = -c_L[1] 
    else: pass # u_s[k] = u_s[k-1]
    u_s = u_s_filter(u_s, cutoff_freq=cutoff_freq, T=T)
    
    theta_d_hebi = np.array([theta[0], theta[1]])
    theta_d_hebi[0] /= m[0]
    theta_d_hebi[0] += u_s[0]
    theta_d_hebi[1] /= m[1]
    theta_d_hebi[1] += u_s[1]

    return theta_d_hebi   


if __name__ == "__main__":
    # call functions before loop:
    theta = np.ones(2) # get theta from get_hebi_feedback
    init_u_s_filter(theta)

    # call before loop if using GPR models
    GPR_number = '4'
    GPR_models = load_GPR_param_models(GPR_number)

    # in loop:
    while True:
        theta = omega = np.ones(2) # get theta, omega from get_hebi_feedback
        theta_d = smooth_backlash_inverse(theta, omega, GPR_models=GPR_models) # recommended to input values for c_R, c_L, m
        # send theta_d to hebi as position command
        print('Test: functions run without errors')
        break
