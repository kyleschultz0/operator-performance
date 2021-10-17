import numpy as np
import matplotlib.pyplot as plt

#=== Lissajous variables ===#
a = 2*np.pi
a_per_b = 0.5
b = a/a_per_b
d = np.pi/4;

def trajectory(t, f):
    xd = np.sin(a*f*t+d)
    yd = np.cos(b*f*t)
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
    return t_max, vel_max
  

# to test functions:
if __name__ == "__main__":
    f = 0.01
    T = 1/f
    t = np.linspace(0, T, 1000)
    pos = trajectory(t, f)

    t_max, vel_max = max_vel(f)
    print("Max velocity:", vel_max)
    print("Time of max velocity:", t_max)

    plt.plot(pos[0, :], pos[1, :])
    plt.show()
   
