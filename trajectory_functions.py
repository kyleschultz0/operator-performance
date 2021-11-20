import numpy as np
import matplotlib.pyplot as plt
window_size = 1000
T = 60

def trajectory(t, f, type):

    if type == "lissajous":
        #=== Lissajous variables ===#
        a = 2*np.pi
        a_per_b = 0.5
        b = a/a_per_b
        d = np.pi/4
        #xd = np.sin(a*f*t+d)
        #yd = np.cos(b*f*t)

    if type == "circle":
        #=== Circle Variables ===#
        w = 2*np.pi*f
        xd = np.cos(w*t)
        yd = np.sin(w*t)
    
    if type == "chirp":
        f0 = 0.001
        f1 = 0.1
        fa = f0 + (f1 - f0) * t / T
        fb = f1 - (f1 - f0) * t / T
        xd = np.cos(np.pi*fa*t)
        yd = np.cos(np.pi*fb*(T-t))

    pos = np.array([xd, yd])
    return pos



def max_vel(f):
    T = 1/f
    dt = 0.001
    t = 0
    r0 = trajectory(0, f, "chirp")
    vel_max = 0
    while t < T-dt:
        t += dt
        drdt = (trajectory(t, f, "chirp") - r0)/dt
        vel = np.sqrt(np.sum(np.square(drdt)))
        r0 = trajectory(t, f, "chirp")
        if vel > vel_max:
            vel_max = vel
            t_max = t
    vel_max = (window_size/2 - 100)*vel_max
    return t_max, vel_max
  
def screen_trajectory(t, f):
    return np.round(window_size/2 - 50 + (window_size/2 - 100)*trajectory(t, f, "chirp"))

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
    T = 60
    t = np.linspace(0, T, 1000)
    pos = trajectory(t, f, "chirp")
    pos_screen = screen_trajectory(t, f)


    #t_max, vel_max = max_vel(f)
    #print("Max velocity:", vel_max)
    #print("Time of max velocity:", t_max)
    #center = trajectory_size(f)
    #print(center)

    plot1 = plt.figure(1)
    plt.plot(pos[0, :], pos[1, :])

    plot2 = plt.figure(2)
    plt.plot(t, pos[0, :])
   
    plot3 = plt.figure(3)
    plt.plot(t, pos[1, :])

    plot4 = plt.figure(4)
    plt.plot(pos_screen[0, :], pos_screen[1, :])


    plt.show()

   
