from math import *
import random
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
 
# landmarks which can be sensed by the robot (in meters)
landmarks = [[30.0, 30.0],[30.0,70.0],[70.0,30.0],[70.0,70.0]]
 
# size of one dimension (in meters)
world_size = 100.0
# The class for modeling robot and particles as well.
class RobotClass:
    """Class for the robot model used in this demo """
    def __init__(self):
        self.x = random.random() * world_size           # robot's x coordinate
        self.y = random.random() * world_size           # robot's y coordinate
        self.orientation = random.random() * 2.0 * pi   # robot's orientation
 
        self.forward_noise = 0.0   # noise of the forward movement
        self.turn_noise = 0.0      # noise of the turn
        self.sense_noise = 0.0     # noise of the sensing
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)

    def sense(self):
        z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)
        return z
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cannot move backwards')
         # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
 
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
 
        # cyclic truncate
        x %= world_size
        y %= world_size
 
        # set particle
        res = RobotClass()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res    
    def measurement_prob(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob
    @staticmethod
    def gaussian(mu, sigma, x):
        """ calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        :param mu:    distance to the landmark
        :param sigma: standard deviation
        :param x:     distance to the landmark measured by the robot
        :return gaussian value
        """
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

def evaluation(r, p):
   
    """Calculate the mean error of the system
    :param r: current robot object
    :param p: particle set
    :return mean error of the system"""
    sum = 0.0
    for i in range(len(p)):
         # the second part is because of world's cyclicity
        dx = (p[i].x - r.x + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        err = sqrt(dx**2 + dy**2)
        sum += err
    return sum / float(len(p))

# The defined function is to be called for rendering animation
def updatelines(t) :
    global p
    global myrobot

    # move the robot and sense the environment after that
    myrobot = myrobot.move(2, 23.)
    z =myrobot.sense()
    # now we simulate a robot motion for each of
    # these particles
    p2 = []
    for i in range(n):
        p2.append( p[i].move(2, 23.) )
 
    p = p2
    # generate particle weights depending on robot's measurement
    w = []
    for i in range(n):
        w.append(p[i].measurement_prob(z))
    # resampling with a sample probability proportional to the importance weight
    p3 = []
 
    index = int(random.random() * n)
    beta = 0.0
    #mw = max(w)
    mw=sum(w)
    for i in range(n):
        #beta += random.random() * 2.0 * mw
        beta +=random.random()*mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % n
        p3.append(p[index])
    p = p3
    
    xs=[p[i].x for i in range(n)]
    ys=[p[i].y for i in range(n)]
    D=np.vstack((xs,ys))
    particles.set_data(D[0:2,:])
    robo_position.set_data([myrobot.x,myrobot.y])

#####End of RobotClass definition##################
    
myrobot = RobotClass()
print myrobot
myrobot.set_noise(5,0.2,2)
myrobot = myrobot.move(0.1, 5.0)

# create a set of 'n' particles
n = 1000  # number of particles
p = []    # list of particles
 
for i in range(n):
    x = RobotClass()
    x.set_noise(5, 0.2, 2)
    p.append(x)

fig,ax = plt.subplots(figsize=(15., 15.))
plt.title('Robot localization with Particle filtering')

# draw coordinate grid for plotting
grid = [0, world_size, 0, world_size]
plt.axis(grid)
plt.grid(b=True, which='major', color='0.75', linestyle='--')
plt.xticks([i for i in range(0, int(world_size), 5)])
plt.yticks([i for i in range(0, int(world_size), 5)])
# Draw landmarks
for lm in landmarks:
    circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
    plt.gca().add_patch(circle)
# Record line objects which will be updated during animation
# 'robo_position': line object for marking robot position
# 'particles': line object for marking 'n' particles
# Plot robot position on top of particles by using 'zorder' parameter.
robo_position,=ax.plot([],[],c='red',marker='*',linestyle="",markersize=15,zorder=2)
particles,=ax.plot([],[],c='green', marker='o',markersize=5,linestyle="",alpha=0.2,zorder=1)


steps = 50  # particle filter steps
    
ani = animation.FuncAnimation(fig, updatelines,steps,
                             interval=1000, blit=False,repeat=False)
# Set up formatting for the movie files

plt.show()
#ani.save('tracking.mp4',writer=animation.FFMpegFileWriter(fps=1))
    
#print 'Step = ', t, ', Evaluation = ', evaluation(myrobot, p)
