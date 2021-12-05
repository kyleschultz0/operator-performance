import numpy as np
import matplotlib.pyplot as plt

class Trajectory:

    def __init__(self, shape, T, f, window_size):
        self.shape = shape
        self.T = T
        self.f = f
        self.window_size = window_size
        self.K = 1  # wrong

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
            #print("f:", self.f)
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

        if self.shape == "chirp2":
            #== Chirp variables ==#
            f0 = 0.001
            f1 = 0.0666
            #=====================#
            fa = f0 + (f1 - f0) * t / self.T
            xd = np.cos(np.pi*fa*t)
            yd = np.cos(np.pi*fa*t)
            K = np.sqrt(np.pi*fa**2 + np.pi*fa**2)
            self.K = K

        if self.shape == "sines":
            #== Sine variables ==#
            f = np.array([0.01, 0.015, 0.02, 0.04, 0.06, 0.08, 0.16, 0.25, 0.35])*1.5
            A = np.array([1, 0.95, 0.83, 0.75, 0.58, 0.25, 0.15, 0.08, 0.04])
            A = 1.3*A/np.sum(A)
            #=====================#
            yd = 0
            for i in range(0,f.shape[0]):
                yd = yd + A[i]*np.cos(2*np.pi*f[i]*t)
            yd = yd - 0.3
            xd = yd

        if self.shape == "varsine":
            #== Sine variables ==#
            max_vel = 0.0254
            A = np.sin(0.1*t) + 0.01
            w = max_vel / A
            #=====================#
            yd = A * np.cos(w*t)
            xd = yd

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
    T = 120
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

   
