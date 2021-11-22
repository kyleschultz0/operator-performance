import numpy as np
import matplotlib.pyplot as plt

class Trajectory:

    def __init__(self, shape, T, f, window_size):
        self.shape = shape
        self.T = T
        self.f = f
        self.window_size = window_size

    def coordinates(self, t):
        if self.shape == "lissajous":
            #== Lissajous variables ==#
            a = 2*np.pi
            a_per_b = 0.5
            b = a/a_per_b
            d = np.pi/4
            #=========================#

            xd = np.sin(a*f*t+d)
            yd = np.cos(b*f*t)

        if self.shape == "circle":
            #== Circle variables ==#
            print("f:", self.f)
            w = 2*np.pi*self.f
            #======================#

            xd = np.cos(w*t)
            yd = np.sin(w*t)
    
        if self.shape == "chirp":
            #== Chirp variables ==#
            f0 = 0.001
            f1 = 0.1
            #=====================#
            fa = f0 + (f1 - f0) * t / self.T
            fb = f1 - (f1 - f0) * t / self.T
            xd = np.cos(np.pi*fa*t)
            yd = np.cos(np.pi*fb*(self.T-t))

        return np.array([xd, yd])          

    def screen_coordinates(self, t):
        return self.window_size/2 - 50 + (self.window_size/2 - 100)*self.coordinates(t)

    def max_vel(self):
        # Compute maximum velocity of trajectory in pixels
        dt = 0.001
        t = 0
        r0 = self.screen_coordinates(0)
        vel_max = 0
        while t < self.T-dt:
            t += dt
            drdt = (self.screen_coordinates(t) - r0)/dt
            vel = np.sqrt(np.sum(np.square(drdt)))
            r0 = self.screen_coordinates(t)
            if vel > vel_max:
                vel_max = vel
                t_max = t

        return t_max, vel_max

# to test functions:
if __name__ == "__main__":
    T = 60
    window_size = 1000
    f = 0.1

    trajectory = Trajectory("chirp", T, f, window_size)

    t = np.linspace(0, T, 1000)
    coords = trajectory.coordinates(t)
    screen_coords = trajectory.screen_coordinates(t)

    t_max, vel_max = trajectory.max_vel()
    print("Max velocity:", vel_max)
    print("Time of max velocity:", t_max)

    plot1 = plt.figure(1)
    plt.plot(coords[0, :], coords[1, :])

    plot2 = plt.figure(2)
    plt.plot(t, coords[0, :])
   
    plot3 = plt.figure(3)
    plt.plot(t, coords[1, :])

    plot4 = plt.figure(4)
    plt.plot(screen_coords[0, :], screen_coords[1, :])


    plt.show()

   
