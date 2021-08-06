import numpy as np
import math

class slip:
    def __init__(self, z, g, s, alpha, fh, sl):     #Initialising the class with required initial parameters
        self.z = z                               #Height of the centre of mass
        self.g = g                               #Gravitational acceleration
        self.s = s                               #Maximum distance of a foot from Centre of Mass
        self.alpha = alpha                       #Inversely proportional to time period
        self.fh = fh                             #Maximum height of the foot end in swing phase
        self.dt = 0                              #Timestep
        self.sl = sl                             #Step length

        self.tc = math.sqrt(self.z/abs(self.g))                      #time constant based on height
        self.ts = 2*self.tc*math.log((1+math.sqrt(1-(self.alpha**2)))/self.alpha)    #time period of one step based on parameters
        self.tb = self.ts/4                                                          #double stance time period, experimental
        self.vex = (self.alpha*self.s*math.sinh(self.ts/(2*self.tc)))/self.tc
        self.w = (2*self.s) + (self.vex*self.tb)
        print(self.ts, "  ", self.tc, self.vex, self.ts/self.tc)

        

        self.base = np.array([0, 0, self.z, 1])         #Base position with respect to world frame
        self.base_v = np.array([0.01,0,0])   #Assuming velocities are same in different frames for straight line motion

        self.fl_tr = np.array([[1,0,0,0],[0,1,0,self.alpha*self.s], [0,0,1,0], [0,0,0,1]])     #transformation matrix for left foot - World Frame
        self.fr_tr = np.array([[1,0,0,0],[0,1,0,-self.alpha*self.s], [0,0,1,0], [0,0,0,1]])    #transformation matrix for right foot - World Frame
        self.fl_tr_inv = np.linalg.inv(self.fl_tr)                       #Inverse transformation matrices   
        self.fr_tr_inv = np.linalg.inv(self.fr_tr)


        self.v_tr = np.array([[1,0,0],[0,1,0], [0,0,1]])                 #Base velocity transformation matrix
        self.v_tr_inv = np.linalg.inv(self.v_tr)


        self.xdf = 0
        self.xf = 0
        self.leg = 0
        self.t_last = 0
        self.t_track = 0
        self.d_track = 0
        self.t = self.ts/2
        self.by = 0
        self.ini = True
        self.stop = False

    def transform_base(self, leg):                                        #Updating position of base in foot's frame
        self.base_v_f = np.matmul(self.v_tr_inv, self.base_v.transpose()).transpose()
        if(leg == 0):
            self.base_fl = np.matmul(self.fl_tr_inv, self.base.transpose()).transpose()
        else:
            self.base_fr = np.matmul(self.fr_tr_inv, self.base.transpose()).transpose()


    def calc_init(self, leg):                   #Calculating initial position of base in foot frame at start of every step                                    
        self.base_v_in = self.base_v_f
        if(leg == 0):
            self.base_in = self.base_fl
        else:
            self.base_in = self.base_fr




    def update_base(self, leg):                               #Updating position on base in world frame based on updated foot frame
        self.base_v = np.matmul(self.v_tr, self.base_v_f.transpose()).transpose()
        if(leg == 0):
            self.base = np.matmul(self.fl_tr, self.base_fl.transpose()).transpose()
        if(leg == 1):
            self.base = np.matmul(self.fr_tr, self.base_fr.transpose()).transpose()


    def update_step(self, leg, t):                           #Updating the position and velocity of the base for each time step
        yi = self.base_in[1]
        yid = self.base_v_in[1]
        xi = self.base_in[0]
        xid = self.base_v_in[0]
        if(leg == 0):
            yf = yi*math.cosh(t/self.tc) + self.tc*yid*math.sinh(t/self.tc)
            yfd = (yi*math.sinh(t/self.tc))/self.tc + yid*math.cosh(t/self.tc)

            xf = xi*math.cosh(t/self.tc) + self.tc*xid*math.sinh(t/self.tc)
            xfd = xi/self.tc*math.sinh(t/self.tc) + xid*math.cosh(t/self.tc)

            self.base_fl[1] = yf
            self.base_fl[0] = xf
            self.base_v_f[0] = xfd
            self.base_v_f[1] = yfd
        
        if(leg == 1):
            yf = yi*math.cosh(t/self.tc) + self.tc*yid*math.sinh(t/self.tc)
            yfd = (yi*math.sinh(t/self.tc))/self.tc + yid*math.cosh(t/self.tc)

            xf = xi*math.cosh(t/self.tc) + self.tc*xid*math.sinh(t/self.tc)
            xfd = xi/self.tc*math.sinh(t/self.tc) + xid*math.cosh(t/self.tc)

            self.base_fr[1] = yf
            self.base_fr[0] = xf
            self.base_v_f[0] = xfd
            self.base_v_f[1] = yfd

        self.update_base(leg)

    def get_z_ellipse(self, x):                                 #Generating an elliptical trajectory for motion of swing leg
        a = (self.foothold[0]-self.foothold_init[0])/2
        b = self.fh
        x = x - (self.foothold_init[0] + a)
        z = math.sqrt((b**2)*(1-min(((x**2)/(a**2)), 1)))
        z = self.foothold_init[2] + z
        return z

    def get_z_line(self, t, init):
        if(init == True):
            if(t<self.ts/4):
                z = self.fh*t*4/self.ts
            else:
                z = 4*self.fh*(0.5-(t/self.ts))
        else:
            if(t<self.ts/2):
                z = self.fh*t*2/self.ts
            else:
                z = 2*self.fh*(1-(t/self.ts))
        return z





    def update_swing(self, leg, t, init):     #Function to update the position of swing leg
        if(init == True):
            x = (self.foothold[0]-self.foothold_init[0])/self.ts*t*2 + self.foothold_init[0]
            y = (self.foothold[1]-self.foothold_init[1])/self.ts*t*2 + self.foothold_init[1]
            if(self.sl == 0):
                z = self.get_z_line(t, init)
            else:
                z = self.get_z_ellipse(x)
        else:
            x = (self.foothold[0]-self.foothold_init[0])/self.ts*t + self.foothold_init[0]
            if(t == self.ts):
                y = self.foothold[1]
            else:
                y = self.foothold_init[1] + (self.base[1] - self.base_init_y)
            if(self.sl == 0):
                z = self.get_z_line(t, init)
            else:
                z = self.get_z_ellipse(x)
        if(leg == 0):
            self.fr_tr[0][3] = x
            self.fr_tr[1][3] = y
            self.fr_tr[2][3] = z
            self.fr_tr_inv = np.linalg.inv(self.fr_tr)
        if(leg == 1):
            self.fl_tr[0][3] = x
            self.fl_tr[1][3] = y
            self.fl_tr[2][3] = z
            self.fl_tr_inv = np.linalg.inv(self.fl_tr)
        #print(self.foothold_init, t)



    def double_stance(self):                      #Updating the base position in double stance phase. Velocities remain constant
        dt = self.dt
        self.base[0] = self.base[0] + self.base_v[0]*dt
        self.base[1] = self.base[1] + self.base_v[1]*dt



    def calc_foothold(self, leg, init, stop, step_length):             #Calculating the next position of foot at the start of the step
        if(leg == 0):
            xi = self.base_fl[0]
        else:
            xi = self.base_fr[0]
        if(init == True):
            self.xf = self.sl/2
            self.xdf = 0
        else:
            self.xf = -xi
            self.xdf = (math.cosh(self.ts/self.tc)+1)*(self.sl/2)/(self.tc*math.sinh(self.ts/self.tc))
        self.base_v_f[0] = self.xdf
        self.update_base(leg)
        self.xfoot = step_length/2 - xi + self.tb*self.xdf
        if(init == True):
            self.xfoot = self.sl + self.tb*self.xdf
        if(stop == True):
            self.xfoot = self.sl/2
            self.stop = True  
        self.sl = step_length
        self.base_init_y = self.base[1]
        if(leg == 0):
            y = -2*self.s - self.tb*self.vex
            self.foothold = np.matmul(self.fl_tr, np.array([self.xfoot,y,0,1]).transpose()).transpose()
            self.foothold_init = np.copy(self.fr_tr.transpose()[3])
        else:
            y = 2*self.s + self.tb*self.vex
            self.foothold = np.matmul(self.fr_tr, np.array([self.xfoot,y,0,1]).transpose()).transpose()
            self.foothold_init = np.copy(self.fl_tr.transpose()[3])




    def step(self, t, stop, step_length):                                            #Updating the step at every time
        t = round(t,2)                                            #Calculating dt based on time difference of calling step
        self.leg_change = False
        self.dt = t-self.t_last
        self.t_last = t
        if(self.t_track==0):                               #At beginning of each new step, initial condition and final foothold are calculated
            self.transform_base(self.leg)
            self.calc_foothold(self.leg, self.ini, stop, step_length)
            self.calc_init(self.leg)
        if(self.t_track<=self.t):                          #Performing the step function till the end of each step's time period
            self.transform_base(self.leg)
            self.update_step(self.leg, self.t_track)
            self.update_swing(self.leg, self.t_track, self.ini)
            self.t_track += self.dt
        else:
            if(self.d_track == 0):                                          #Clearing any residual errors
                if(t<self.ts):
                    self.update_swing(self.leg, self.ts/2, True)
                else:
                    self.update_swing(self.leg, self.ts, self.ini)
                if(self.leg == 0):                                    #Clearing residual errors due to tme mismatch in the leg's position and velocity
                    self.base_fl[1] = -self.s
                    self.base_v_f[1] = -self.vex
                    self.base_fl[0] = self.xf
                    self.base_v_f[0] = self.xdf
                else:
                    self.base_fr[1] = self.s
                    self.base_v_f[1] = self.vex
                    self.base_fr[0] = self.xf
                    self.base_v_f[0] = self.xdf
                self.update_base(self.leg)
                self.by = self.base[1]
                self.bx = self.base[0]
            
            if(self.d_track<=self.tb):            #Performing the double stance phase for it's time period
                self.double_stance()
                self.d_track += self.dt
            else:
                if(t<=self.ts/2):                  #Changing the initial step condition as initial step only runs for half the time period
                    self.ini = True
                else:
                    self.ini = False
                if(self.ini == False):
                    self.t = self.ts
                self.leg_change = True                 #Changing the leg in swing phase and clearing residual errors
                if(self.leg==0):
                    self.leg = 1
                    self.base[1] = self.by - self.vex*self.tb
                    self.base[0] = self.bx + self.xdf*self.tb
                else:
                    self.leg = 0
                    self.base[1] = self.by + self.vex*self.tb
                    self.base[0] = self.bx + self.xdf*self.tb

                self.t_track = 0
                self.d_track = 0

        self.base_tr = np.array([[1,0,0,self.base[0]],[0,1,0,self.base[1]], [0,0,1,self.base[2]], [0,0,0,1]]) #Forming base transformation matix
        base_tr = np.copy(self.base_tr)     
        fl_tr = np.copy(self.fl_tr)                        #Returning transformation matrix of base, left foot and right foot and base velocity
        fr_tr = np.copy(self.fr_tr)                         #Also rturning the leg in swing phase, 0 for right and 1 for left
        base_v = np.copy(self.base_v)                       #Returning a parameter which is True for the step in which the swing leg changes
        return (base_tr, fl_tr, fr_tr, self.leg, base_v)   #returning all the data as a single tuple.
 
