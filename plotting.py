import numpy as np
import matplotlib.pyplot as plt

class robot:
    """Robot objects"""
    def __init__(self, start, end):
        self.start = start
        self.end = end
        #self.position = [self.start, self.end] ... wrong

    def update_position(self):
        pass

class obsticle:
    """unmovable obsticles that the robots must avoid.. maybe can make dynamic obsticles??"""
    def __init__(self, location, size=2, shape='square'):
        self.location = location  # locaton of bottom left point of square for now
        self.size = size
        self.shape = shape
        if shape == 'square':
            self.grid_points = []
            #Hard code for now
            self.grid_points = [(5,5,6,6),(5,6,5,6)]

            #for i in range(self.size*self.size):
             #   self.location # these are the grid points that are taken up by the obsticle


# temporarily hard code grid world size = 10 by 10 "squares"
grid_size = np.array([10,10])
print grid_size

obsticle1 =  obsticle((5,5))
print "obsticle location", obsticle1.grid_points



robot1 = robot([0,0], [4,3])
robot2 = robot([9,3], [2, 10])

print "Robot 1's starting position: ", robot1.start

# make graph

plt.scatter(robot1.start[0], robot1.start[1], c='b')
plt.scatter(robot2.start[0], robot2.start[1], c='b', label='Robot Start Points')
plt.scatter(robot1.end[0],robot1.end[1], c='r')
plt.scatter(robot2.end[0],robot2.end[1], c='r', label='Robot End Points')
plt.scatter(obsticle1.grid_points[0], obsticle1.grid_points[1], c='k', label="Obstacle")
plt.xlim(-1,11)
plt.ylim(-1,11)
plt.yticks(np.arange(0, 11, step=1))
plt.xticks(np.arange(0, 11, step=1))
plt.text(robot1.start[0], robot1.start[1], "Robot1 Start")
plt.text(robot1.end[0],robot1.end[1], "Robot1 End")
plt.legend()
plt.grid()
plt.show()
