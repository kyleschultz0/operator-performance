import tkinter
from time import time, sleep
from trajectory_functions import *


f = 0.1
# curve frequency
T = 1/f

window_size = 1000
# width of the animation window
animation_window_width = window_size
# height of the animation window
animation_window_height = window_size
# radius of the ball
target_ball_radius = 10
# radius of the encoder ball
input_ball_radius = 5
# delay between successive frames in seconds
animation_refresh_seconds = 0.01

class Ball:
    def __init__(self, position, radius, color, canvas):
        self.pos = position
        self.radius = radius
        self.color = color
        self.canvas = canvas
        self.ball = self.canvas.create_oval(self.pos[0]-self.radius,
                                            self.pos[1]-self.radius,
                                            self.pos[0]+self.radius,
                                            self.pos[1]+self.radius,
                                            fill = self.color)

    def move(self, position):
        self.pos = position
        self.canvas.coords(self.ball,
                      self.pos[0]-self.radius,
                      self.pos[1]-self.radius,
                      self.pos[0]+self.radius,
                      self.pos[1]+self.radius)

def create_animation_window():
  window = tkinter.Tk()
  window.title("Tkinter Animation Demo")
  window.geometry(f'{animation_window_width}x{animation_window_height}')
  return window
 
# Create a canvas for animation and add it to main window
def create_animation_canvas(window):
  canvas = tkinter.Canvas(window)
  canvas.configure(bg="black")
  canvas.pack(fill="both", expand=True)
  return canvas



if __name__ == "__main__":

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)
    pos = np.round(window_size/2 - 50 + (window_size/2 - 100)*trajectory(0, f))

    target_ball = Ball(pos, target_ball_radius, "red", animation_canvas)
    input_ball = Ball(pos, input_ball_radius, "white", animation_canvas)


    t0 = time()

    while True:
        t = time() - t0
        pos = np.round(window_size/2 - 50 + (window_size/2 - 100)*trajectory(t, f))
        target_ball.move(pos)
        animation_window.update()

        if t > (T):
            print("Stopping: trajectory completed")
            break
