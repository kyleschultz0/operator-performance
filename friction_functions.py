import pickle
import numpy as np

# Initialize ff friction parameters:
ff_fric = np.zeros(2)
f1k_1 = 0 # f1[k-1] - output
g1k_1 = 0 # g1[k-1] - input
f2k_1 = 0 # f2[k-1] - output
g2k_1 = 0 # g2[k-1] - input

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

def fric_comp_filter(g, cutoff_freq, T=0.01):
    global f1k_1
    global g1k_1
    global f2k_1
    global g2k_1

    tau = 1/(2*np.pi*cutoff_freq) 
    f1 = (g[0] + g1k_1 - (1-2*tau/T)*f1k_1)/(2*tau/T+1)
    f2 = (g[1] + g2k_1 - (1-2*tau/T)*f2k_1)/(2*tau/T+1)
    f1k_1 = f1 
    g1k_1 = g[0]
    f2k_1 = f2 
    g2k_1 = g[1]
    return np.array([f1,f2])

def smooth_fric_comp(theta_d, omega_d, GPR_models=None,
                     cutoff_freq=0.85, T=0.01,
                     fsp=[0,0], fsn=[0,0], B0=[0,0]):
    global ff_fric

    if GPR_models != None:
        fsp[1] = GPR_models[3].predict(theta_d[0].reshape(1,-1), return_std=False)[0]
        fsn[1] = GPR_models[4].predict(theta_d[0].reshape(1,-1), return_std=False)[0]
        B0[1] = GPR_models[5].predict(theta_d[0].reshape(1,-1), return_std=False)[0]

    effort = np.zeros(2)
    if omega_d[0] > 0:
        ff_fric[0] = fsp[0]
    elif omega_d[0] < 0:
        ff_fric[0] = fsn[0]
    else: pass # ff_fric[k] = ff_fric[k-1]
    if omega_d[1] > 0:
        ff_fric[1] = fsp[1]
    elif omega_d[1] < 0:
        ff_fric[1] = fsn[1]
    else: pass # ff_fric[k] = ff_fric[k-1]
    effort = fric_comp_filter(ff_fric, cutoff_freq=cutoff_freq, T=T)
    effort += np.array([B0[0]*omega_d[0], B0[1]*omega_d[1]])
    return effort

if __name__ == "__main__":

    # call before loop if using GPR models
    GPR_number = '4'
    GPR_models = load_GPR_param_models(GPR_number)

    # in loop:
    while True:
        theta = omega = np.ones(2) # get theta, omega from get_hebi_feedback
        theta_d = smooth_fric_comp(theta, omega, GPR_models=GPR_models) # recommended to input values for fsp, fsn, B0
        # send theta_d to hebi as position command
        print('Test: functions run without errors')
        break
