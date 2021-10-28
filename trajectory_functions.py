import numpy as np
import matplotlib.pyplot as plt
window_size = 1000

def trajectory(t, f):

    #=== Lissajous variables ===#
    a = 2*np.pi
    a_per_b = 0.5
    b = a/a_per_b
    d = np.pi/4
    #xd = np.sin(a*f*t+d)
    #yd = np.cos(b*f*t)

    #=== Circle Variables ===#
    w = 2*np.pi*f
    xd = np.cos(w*t)
    yd = np.sin(w*t)
    
    pos = np.array([xd, yd])
    return pos

def max_vel(f):
    T = 1/f
    dt = 0.001
    t = 0
    r0 = trajectory(0, f)
    vel_max = 0
    while t < T-dt:
        t += dt
        drdt = (trajectory(t, f) - r0)/dt
        vel = np.sqrt(np.sum(np.square(drdt)))
        r0 = trajectory(t, f)
        if vel > vel_max:
            vel_max = vel
            t_max = t
    vel_max = (window_size/2 - 100)*vel_max
    return t_max, vel_max
  
def screen_trajectory(t, f):
    return np.round(window_size/2 - 50 + (window_size/2 - 100)*trajectory(t, f))

def trajectory_size(f):
    T = 1/f
    dt = 0.001
    t = 0
    xmin = ymin = ymax = xmax = 0
    while t < T-dt:
        t += dt
        pos = screen_trajectory(t, f)
        x = pos[0]
        y = pos[1]
        if x > xmax:
            xmax = x
        if y > ymax:
            ymax = y
        if x < xmin:
            xmin = x
        if y < ymin:
            ymin = y
    center = np.array([xmax - xmin, ymax - ymin])/2
    return center


# to test functions:
if __name__ == "__main__":
    f = 0.01
    T = 1/f
    t = np.linspace(0, T, 1000)
    pos = trajectory(t, f)

    t_max, vel_max = max_vel(f)
    print("Max velocity:", vel_max)
    print("Time of max velocity:", t_max)
    center = trajectory_size(f)
    print(center)
    plt.plot(pos[0, :], pos[1, :])
    plt.show()
   
