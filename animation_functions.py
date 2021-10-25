import tkinter
from time import time, sleep
import numpy as np
from trajectory_functions import screen_trajectory


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
 
# create circle on Tk canvas:
def add_circle(x, y, r, canvasName, color="white"): #center coordinates, radius, canvas, color
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvasName.create_oval(x0, y0, x1, y1, fill=color)

# Create a canvas for animation and add it to main window
def create_animation_canvas(window):
  canvas = tkinter.Canvas(window)
  canvas.configure(bg="black")
  canvas.pack(fill="both", expand=True)

  # replace with arbitrary trajectory follower
  # why isnt it centered?
  circle_linew = 2
  add_circle(450, 450, 400+circle_linew/2, canvas, "white")
  add_circle(450, 450, 400-circle_linew/2, canvas, "black")
  return canvas



if __name__ == "__main__":

    animation_window = create_animation_window()
    animation_canvas = create_animation_canvas(animation_window)
    pos = screen_trajectory(0, f)

    target_ball = Ball(pos, target_ball_radius, "red", animation_canvas)
    input_ball = Ball(pos, input_ball_radius, "white", animation_canvas)


    t0 = time()

    while True:
        t = time() - t0
        pos = screen_trajectory(t, f)
        target_ball.move(pos)
        animation_window.update()

        if t > (T):
            print("Stopping: trajectory completed")
            break
