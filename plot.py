from slip_sl import slip
import time
import numpy as np
import matplotlib.animation as animation         #Plotting a animated 2D graph to verify our code
import matplotlib.pyplot as plt

z = 0.8
g = -9.8                                    #Setting the parameters and passing them in the model
step_y = 0.2
alpha = 0.6
fh = 0.05
step_length = 0.1

model = slip(z, g, step_y, alpha, fh, step_length)    #Initailaising our slip model

dt = 0.01

fig = plt.figure()                                     
ax = fig.add_subplot(111, aspect='equal', autoscale_on = False, xlim = (0, 40*step_length), ylim=(-10*step_y,10*step_y))
ax.grid()

line, = ax.plot([], [], 'o-', lw = 2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
step_text = ax.text(0.63, 0.95, '', transform=ax.transAxes)
t = 0

def init():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')
    step_text.set_text('')
    return line, time_text, step_text

def animate(i):
    """perform animation step"""
    global model, t, dt, step_length
    if(t>5):   #Changing step length.
        step_length = 0.15
    if(t>8):    #Changing step length again.
        step_length = 0.05
    up = model.step(t, False, step_length)                        #Updateing the coordinates at each time step
    up = [[up[0][0][3], up[1][0][3], up[2][0][3]], [up[0][1][3], up[1][1][3], up[2][1][3]]]  #Passing x,y coordinates to be updated in the plot
    t = t+dt
    time.sleep(dt/2)
    line.set_data(up)
    time_text.set_text('time = %.1f' % t)
    step_text.set_text('step_length = %.2f' % step_length)
    return line, time_text, step_text

ani = animation.FuncAnimation(fig, animate, frames=1000, interval=dt, blit=True, init_func=init)   #running the plot
plt.show()                         #Displaying the plot
