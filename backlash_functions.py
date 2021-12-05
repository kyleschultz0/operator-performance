import pickle
import numpy as np

### global variables for hammerstein switching states ###

# [joint1_state, joint2_state]
# state == 0: positive joint velocity
# state == 1: negative joint velocity
# initialized to match "resetting backlash"
bl_states = np.zeros(2)
bl_switch_timer = np.zeros(2)

def inverse_hammerstein(theta_d, omega_d, omega_f, GPR_models=None,
                        c=[0,0,0,0], K=[1.0, 1.0], tau=[0,0,0,0],
                        threshold=0.0, switch_freq=1, T=0.01):
    theta_d_hebi = np.zeros(2)
    if omega_f[0] > 0:
        theta_d_hebi[0] = (tau[0]*omega_d[0] + theta_d[0]) / K[0]
        theta_d_hebi[0] += c[0]
    elif omega_f[0] < 0:
        theta_d_hebi[0] = (tau[1]*omega_d[0] + theta_d[0]) / K[0]
        theta_d_hebi[0] += c[1]
    if omega_f[1] > 0:
        theta_d_hebi[1] = (tau[2]*omega_d[1] + theta_d[1]) / K[1]
        theta_d_hebi[1] += c[2]
    elif omega_f[1] < 0:
        theta_d_hebi[1] = (tau[3]*omega_d[1] + theta_d[1]) / K[1]
        theta_d_hebi[1] += c[3]

    #global bl_states
    #global bl_switch_timer
    #if bl_switch_timer[0] > 1 / switch_freq:
    #    if omega_d[0] > threshold:
    #        if bl_states[0] != 0: bl_switch_timer[0] = 0
    #        bl_states[0] = 0
    #    elif omega_d[0] < -threshold:
    #        if bl_states[0] != 1: bl_switch_timer[0] = 0
    #        bl_states[0] = 1
    #if bl_switch_timer[1] > 1 / switch_freq:
    #    if omega_d[1] > threshold:
    #        if bl_states[1] != 0: bl_switch_timer[1] = 0
    #        bl_states[1] = 0
    #    elif omega_d[1] < -threshold:
    #        if bl_states[1] != 1: bl_switch_timer = 0
    #        bl_states[1] = 1
    #bl_switch_timer += T

    #theta_d_hebi = np.zeros(2)
    #if bl_states[0] == 0:
    #    theta_d_hebi[0] = (tau[0]*omega_d[0] + theta_d[0]) / K[0]
    #    theta_d_hebi[0] += c[0]
    #elif bl_states[0] == 1:
    #    theta_d_hebi[0] = (tau[1]*omega_d[0] + theta_d[0]) / K[0]
    #    theta_d_hebi[0] += c[1]
    #if bl_states[1] == 0:
    #    theta_d_hebi[1] = (tau[2]*omega_d[1] + theta_d[1]) / K[1]
    #    theta_d_hebi[1] += c[2]
    #elif bl_states[1] == 1:
    #    theta_d_hebi[1] = (tau[3]*omega_d[1] + theta_d[1]) / K[1]
    #    theta_d_hebi[1] += c[3]

    return theta_d_hebi
