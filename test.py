from slip_sl import slip
import time
import numpy as np

#Testing the code
z = 0.8
g = -9.8
step_y = 0.2                    #Parameters passed, check their information in the slip class
alpha = 0.6
fh = 0.05
step_length = 0.1

model = slip(z, g, step_y, alpha, fh, step_length)       #Model is formed

start = time.time()
i = 0

while(True):
    print(model.step(i, False, step_length))                                 #The step is called periodically to return the required infromation
    time.sleep(0.01)
    i = i+0.01
