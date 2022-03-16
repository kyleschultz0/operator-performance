import numpy as np
import matplotlib.pyplot as plt

class Trajectory:

    def __init__(self, shape, T, f, window_size, trial = 1):
        self.shape = shape
        self.T = T
        self.f = f
        self.window_size = window_size
        self.K = 1  # wrong
        self.trial = trial
        
        #== Trajectory initialization ==#
        t = np.linspace(0, T, 100*T)
        yd = np.zeros(t.shape)
        xd = np.zeros(t.shape)

        if self.shape == "sines":
            #== Sine variables ==#
            wi = 2*np.pi/60;
            wi = 2*np.pi/100;
            kt = np.array([[1, 2, 3, 4, 5, 7, 8, 10, 11, 19, 20, 27, 28, 32, 36, 37, 44, 45, 55, 56]])
            f = wi*kt

            A1 = np.array([[0.610, 0.610, 0.610, 0.610, 0.610, 0.610, 0.610, 0.610]])
            A2 = np.array([[0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145,
                  0.145, 0.145, 0.145]])
            A = np.concatenate((A1, A2), axis=1, out=None, dtype=None, casting="same_kind")
            #=====================#

            for i in range(0,f.shape[1]):
                xd += A[0, i]*np.sin(f[0, i]*t)

            xd *= 1.6/np.sum(A)
            xd -= xd[0]
            
        self.t = t
        self.xd = xd
        self.yd = yd
        
    def coordinates(self, t):
        return np.array([np.interp(t, self.t, self.xd), np.interp(t, self.t, self.yd)])

    def screen_coordinates(self, t):
        return self.window_size/2 - 50 + (self.window_size/2 - 100)*self.coordinates(t)

    def max_vel(self):
        # Compute maximum velocity of trajectory in pixels
        dt = 0.001
        t = 0
        r0 = self.screen_coordinates(0)
        vel_max = 0
        t_max = 0
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
    trial = 3

    trajectory = Trajectory("sines", T, f, window_size, trial)
    # trajectory.trialLookup()

    t = np.linspace(0, T, 1000)
    coords = trajectory.coordinates(t)
    screen_coords = trajectory.screen_coordinates(t)

    print(trajectory.yd)
    print(trajectory.xd)
    print(coords)
    print(screen_coords)

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

    plot5 = plt.figure(5)
    plt.plot(t, screen_coords[0, :])

    plt.show()

   
