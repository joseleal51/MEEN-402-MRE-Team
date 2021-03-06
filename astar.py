from __future__ import print_function
import matplotlib.pyplot as plt
from math import sqrt
import numpy

import matplotlib.animation as animation


class AStarGraph(object):
    # This class creates an object that is the grid-world with the barries

    def __init__(self, world_size, barrier_points):
        self.x_size = world_size[0]
        self.y_size = world_size[1]
        self.barriers = barrier_points
        #print("type: ", type(self.barriers))

    def heuristic(self, start, goal):
        # Use Chebyshev distance heuristic if we can move one square either
        # adjacent or diagonal
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        h = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy) # reduces dx + dy - min(dx, dy) for D=D2=1

        # Use euclidean distance heruistic
        #h = sqrt(dx*dx + dy*dy)
        #print('\nh:', h, ' start:', start, ' goal:', goal, '\n')
        return h

    def get_vertex_neighbours(self, pos):
        n = []
        # Moves allowed like a chess king
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > self.x_size or y2 < 0 or y2 > self.y_size:
                continue
            n.append((x2, y2))
            #print('get_vertex_neighbours: (x2, y2)', (x2, y2))
        return n

    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b in barrier:
                return 50  # High cost to enter barrier squares
        # check if moving diagonal
        if a[0] != b[0] and a[1] != b[1]: # then a and b are diagonal
            #print('Diagonal\ta: ',a, '\tb: ', b)
            return 1.4142 # ~sqrt(2)
        return 1  # up or down movement

# F = G + H

class AStarSearch:
    """Class of functions for A* search algorithm"""
    def __init__(cls):
        pass

    @classmethod
    def solve(cls, start, end, graph):
        G={}  # Actual movement cost to each position from the start position
        F={}  # Estimated movement cost of start to end going via this position

        # *** each node has location - need to add heading to add a turning cost
        
        # Initialize starting values
        G[start]=0
        F[start]=graph.heuristic(start, end)

        closedVertices=set()
        openVertices=set([start])
        cameFrom={}

        while len(openVertices) > 0:
            # Get the vertex in the open list with the lowest F score
            current=None
            currentFscore=None
            for pos in openVertices:
                if current is None or F[pos] < currentFscore:
                    currentFscore=F[pos]
                    current=pos

            # Check if we have reached the goal
            if current == end:
                # Retrace our route backward
                path=[current]
                while current in cameFrom:
                    current=cameFrom[current]
                    path.append(current)
                path.reverse()
                return path, F[end]  # Done!

            # Mark the current vertex as closed
            openVertices.remove(current)
            closedVertices.add(current)

            # Update scores for vertices near the current position
            for neighbour in graph.get_vertex_neighbours(current):
                if neighbour in closedVertices:
                    continue  # We have already processed this node exhaustively
                candidateG=G[current] + graph.move_cost(current, neighbour)

                if neighbour not in openVertices:
                    openVertices.add(neighbour)  # Discovered a new vertex
                elif candidateG >= G[neighbour]:
                    continue  # This G score is worse than previously found

                # Adopt this G score
                cameFrom[neighbour]=current
                G[neighbour]=candidateG
                H=graph.heuristic(neighbour, end)
                F[neighbour]=G[neighbour] + H
                #print('F: ', F)

        raise RuntimeError("A* failed to find a solution")

class ScheduleRobots:
    """ Take the initial robot paths and determine when, where and which robots collide. Then determine which robots
    need to 'wait' so that the collisions will not happen. The output will be the new discritized points that the robots
    will occupy in time"""
    def __init__(self, routes):
        self.longest_route_len = max([len(x) for x in routes])
        self.time = self.longest_route_len
        self.num_robots = len(routes)

        #self.priorities = 

        #print('yoyo', self.longest_route_len)

        """ Transpose paths to time - becasue the routes input is a list of list where the 
            first list is the route of one robot. Example: [[robot 1 route in tuple's of points],
            [robot 2 route in tuple's of points]]
            After being transposted the first list in the list is the first point of each robot
            Example: [[robo1 point 1, robo2 point 1], [robo1 point 2, robo2 point 2]]
        """

        self.routes_time = [[(0,0)]*self.num_robots for t in range(self.time)]
        for t in range(self.time):
            for robo in range(self.num_robots):
                try:
                    self.routes_time[t][robo] = routes[robo][t]
                except: # the route for a robot is shorter than the other ones
                    # make the robot stay at its end point
                    #print("exception handled")
                    self.routes_time[t][robo] = self.routes_time[t-1][robo]     


        #r [[robo1 route (0,0),(1,1),(2,2)], [robo2 route]]
        #print('7 ???: ', self.routes_time[7][0][0])
        print('\nRoutes')


        #Defining Priorities
        routes_length = [len(j) for j in routes]
        #routes_length = [9, 13, 9, 13, 10]
        routes_length = numpy.array(routes_length)

        temp = routes_length.argsort()
        inverse_priority_list= temp.argsort()
        #inverse_priority_list = [r1_inv_priority, r2_inv_priority, r3_inv_priority, r4_inv_priority, r5_inv_priority]
        #print("Priority List = ", inverse_priority_list)



        # NEED TO FIND A WAY TO INVERT THE PRIORITY ORDER





        for i in range(len(self.routes_time)): print(self.routes_time[i], 'T = ',i)


        # Find collisons
        self.collisions = {}
        for i, locations in enumerate(self.routes_time):
            a = [locations.count(j) for j in locations]
            print('T= ',i, 'Count of each point in time (2 means collision)', a)
        
            if max([locations.count(j) for j in locations]) > 1:
                print(max([locations.count(j) for j in locations]))
                print('COLLISION at t=', i)
                self.collisions[i] = locations[locations.index(max(locations))]
                print('collisions: ', self.collisions)


class robot:
    '''doc string'''
    def __init__(self, start_point, end_point):
        self.start_point = start_point
        self.end_point = end_point
        self.path = None            # Original A* output
        self.mod_path = None        # List of points that correspond to where the robot should be at each time
        self.priority = None
        self.steps = None


def main_calculation(
    barrier_points=[[(3,4),(2, 4), (2, 5), (2, 6), (3, 6), (4, 6),
                     (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)],
                     [(7,9),(8,9)]],
    world_size = (10, 10),
    robots_starts = [(0,0), (8, 10), (3,5), (9,9), (7,7)],
    robots_ends = [(8,4), (1,1), (7,3), (4,3), (2,2)]):
    
    num_robots = len(robots_starts)

    # Create robot objects and store in a dictionary where the first robot is the 0th robot
    robots = {}
    for i in range(num_robots):
        robots[i] = robot(robots_starts[i], robots_ends[i])

    print('robots dictionary: ', robots, '\nRobot 1 priority: ', robots[0].priority)
    graph = AStarGraph(world_size, barrier_points)
    results = []
    costs = []

    for i in range(num_robots):
        result, cost = AStarSearch.solve(robots[i].start_point, robots[i].end_point, graph)
        robots[i].path = result
        robots[i].steps = len(results)
        results.append(result)
        costs.append(cost)
    print('paths:')
    for r in results: print(len(r), '\n', r, '\n\n')

    for i in range(num_robots)
    robots[i].priority = np.array([len(v) for v in results]).argsort().argsort()[i] for i in 


    sim = ScheduleRobots(results, robots)


    
    # animation
    fig = plt.figure(num='MRE Simulaion', figsize=(10,6)) 
    ax1 = fig.add_subplot(1,1,1)
    data = sim.routes_time
    collisions = sim.collisions
    #print(['Robo1'*num_robots])
    colors = {0:'k', 1:'r', 2:'g', 3:'b', 4:'c', 5:'y', 6:'g'}
 
    def animate(i):
        ax1.clear()
        for barrier in graph.barriers:
        #print('\nBarrier: ', barrier)
            x = [v[0] for v in barrier]
            y = [v[1] for v in barrier]
            ax1.plot(x, y, 'k')
            
        try: points = data[i]
        except: # exception will be raised when animation frames are longer than the data
            points = data[-1] # hold the last location of the robots

        for j, robo in enumerate(points):
            #print('robo: ', robo, 'j: ', j)
            ax1.scatter(robo[0], robo[1], s=100, c=colors[j])
        ax1.grid()
        ax1.set_xlim(-1, world_size[0]+1)
        ax1.set_ylim(-1, world_size[1]+1)
        ax1.set_xticks(list(range(world_size[0]+1)))
        ax1.set_yticks(list(range(world_size[1]+1)))
        if i < sim.time: ax1.set_xlabel('T = '+str(i),fontsize=20) 
        else: ax1.set_xlabel('T = '+str(sim.time-1),fontsize=20)
        #ax1.set_ylabel('Y',fontsize=20)
        ax1.set_title('Mult-Agent Path Planning - Simulation')
        ax1.grid(1)

        try: 
            ax1.scatter(collisions[i][0], collisions[i][1], 300, facecolors='none', edgecolors='r')
            ax1.text(collisions[i][0]+0.5, collisions[i][1], "COLLISION")
        except: pass

        for j, result in enumerate(results):
            ax1.plot([x[0] for x in result], [x[1] for x in result], c=colors[j] )
        for i in range(num_robots):
            ax1.scatter(robots_starts[i][0],robots_starts[i][1],s=40,c=colors[i])
            ax1.scatter(robots_ends[i][0],robots_ends[i][1],s=40,c=colors[i])
            ax1.text(robots_starts[i][0],robots_starts[i][1],"Start "+str(i+1))
            ax1.text(robots_ends[i][0],robots_ends[i][1],"End"+str(i+1)) 

    ani = animation.FuncAnimation(fig, animate, interval=600, frames=sim.time+5, repeat=True)
    plt.show()
    ani.save('MRE_with_collisions.gif', writer='pillow')

# animate graph - make time := distance (need to define a min distance/time that robots can get to eachother (collison avoidance))


if __name__ == "__main__":
    main_calculation()

