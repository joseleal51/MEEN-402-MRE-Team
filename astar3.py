from __future__ import print_function
import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

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
        #D = 1
        #D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        #h = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy) # reduces dx + dy - min(dx, dy) for D=D2=1

        # Use euclidean distance heruistic
        h = sqrt(dx*dx + dy*dy)
        #print('\nh:', h, ' start:', start, ' goal:', goal, '\n')
        return h

    def get_vertex_neighbours(self, pos):
        n = []
        # Moves allowed like a chess king
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
        #for dx, dy in [(0.1, 0), (-0.1, 0), (0, 0.1), (0, -0.1), (0.1, 0.1), (-0.1, 0.1), (0.1, -0.1), (-0.1, -0.1)]:
        #for dx, dy in [(0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5), (0.5, 0.5), (-0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]:
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
        self.routes = routes
        self.time = max([len(x) for x in routes])
        self.num_robots = len(routes)
        self.collisions_points = {}
        self.priorities = list(np.array([len(j) for j in routes]).argsort().argsort())
        self.bool_collision = True  # by default there a collision is detected unless proven otherwise

    def within_one_node(self, points_list):
        points = points_list.copy()
        self.close_robots = []
        flag = False
        for robo in range(len(points)):
            try: p = points.pop(0)
            except: print('*Failed* ', points)
            for i, pnt in enumerate(points):
                if pnt[0]+1==p[0] or pnt[0]-1==p[0] or pnt[0]==p[0]: # 1st check if x point is within a distance of 1
                    if pnt[1]+1==p[1] or pnt[1]-1==p[1] or pnt[1]==p[1]: # 2nd check if y point is within dist = 1
                        # then the 2 points are next to each other
                        self.close_robots.append((robo,i+robo+1)) # add the first & 2nd robo's index
                        flag = True
        return flag
            
    def find_collisions(self):
        """This method will identify when, where and which robots collide"""
        self.collisions_points = {} #
        colliding_robots = []
        for t in range(self.time):
            pnts_in_time = []
            for robo in range(self.num_robots):
                try: pnts_in_time.append(self.routes[robo][t])
                except: pass
            
            count_of_pnts = [pnts_in_time.count(j) for j in pnts_in_time] 

            # 1st check if two robots want to occupy the same point at the same time
            if max(count_of_pnts) > 1:
                # then there is a collision becasue two paths want to occupy the same point in time
                for i, count in enumerate(count_of_pnts):
                    if count > 1:
                        colliding_robots.append(i)
                    try:
                        self.collisions_points[t] = self.routes[colliding_robots[0]][t] # need to figure how to to store multiple iterations of collision avoidance
                    except: pass
                return t, colliding_robots
            
            # 2nd check if two robots want swap points (to cross each other)
            elif self.within_one_node(pnts_in_time): # to be added.. will have to use a t-1 or t+1 to see if points "switch"
                #print('t=',t, self.close_robots, 'close robots')
                # check robots switch pathes
                for robo1, robo2 in self.close_robots:
                    try:
                        if self.routes[robo1][t+1] == self.routes[robo2][t]:
                            if self.routes[robo2][t+1] == self.routes[robo1][t]:
                                print("1ROBOT: ", robo1+1, ' & ', robo2+1, ' crossed between t=', t, '& ', t+1)
                                colliding_robots.append(robo1)
                                colliding_robots.append(robo2)
                                return t, colliding_robots
                    except:
                        try:
                            if self.routes[robo1][t] == self.routes[robo2][t+1]:
                                if self.routes[robo2][t] == self.routes[robo1][t+1]:
                                    print("2ROBOT: ", robo1+1, ' & ', robo2+1, ' crossed between t=', t, '& ', t+1)
                                    colliding_robots.append(robo1)
                                    colliding_robots.append(robo2)
                                    return t, colliding_robots
                        except: pass

        self.bool_collision = False
        return t, [] # no robots colliding

    def wait_heuristic(self, colliding_robots):
        """This method will attempt to avoid collisions by making the lower priority robot wait"""
        if len(colliding_robots) == 0: return
        else:
            if self.priorities[colliding_robots[0]] > self.priorities[colliding_robots[1]]:
                self.routes[colliding_robots[1]].insert(0,self.routes[colliding_robots[1]][0])
            else: 
                self.routes[colliding_robots[0]].insert(0,self.routes[colliding_robots[0]][0])

    def create_simulation_data(self):
        data = [[(0,0)]*self.num_robots for t in range(self.time)]
        for t in range(self.time):
            for robo in range(self.num_robots):
                try:
                    data[t][robo] = self.routes[robo][t]
                except: # the route for a robot is shorter than the other ones
                    # make the robot stay at its end point
                    #print("exception handled")
                    data[t][robo] = data[t-1][robo]
        return data


def main_calculation(
    barrier_points=[[(3,4),(2, 4), (2, 5), (2, 6), (3, 6), (4, 6),
                     (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)],
                     [(7,9),(8,9)]],
    world_size = (10, 10),
    robots_starts = [(0,0), (8, 10), (3,5), (9,9), (7,7)],
    robots_ends = [(1,9), (1,1), (9,2), (4,3), (2,2)]):
    
    num_robots = len(robots_starts)

    graph = AStarGraph(world_size, barrier_points)
    results = []
    costs = []

    for i in range(num_robots):
        result, cost = AStarSearch.solve(robots_starts[i], robots_ends[i], graph)
        results.append(result)
        costs.append(cost)
 
    sim = ScheduleRobots(results)

    avoid_collisions = True
    if avoid_collisions:
        loop_count = 0
        while sim.bool_collision:
            t, colliding_robots = sim.find_collisions()
            print('loop count: ', loop_count+1, ' colliding_robots: ', colliding_robots, 't=',t)
            sim.wait_heuristic(colliding_robots)
            loop_count += 1
            if loop_count > 50: print('max iterations'); break
    else: sim.find_collisions()

    data = sim.create_simulation_data()

    # animation
    fig = plt.figure(num='MRE Simulaion', figsize=(10,6)) 
    ax1 = fig.add_subplot(1,1,1)
    collisions = sim.collisions_points
    colors = {0:'k', 1:'r', 2:'g', 3:'b', 4:'c', 5:'y', 6:'g'}
 
    def animate(i):
        ax1.clear()
        for barrier in graph.barriers:
        #print('\nBarrier: ', barrier)
            x = [v[0] for v in barrier]
            y = [v[1] for v in barrier]
            ax1.plot(x, y, 'k')
        
        # # Hardcode!!!!!!!!
        # if i == 0 or i == 1:
        #     ax1.scatter(7,7, 300, facecolors='none', edgecolors='r')
        #     ax1.text(7+0.5, 7-0.5, "WAIT")

        try: points = data[i]
        except: # exception will be raised when animation frames are longer than the data
            points = data[-1] # hold the last location of the robots

        for j, robo in enumerate(points):
            ax1.scatter(robo[0], robo[1], s=100, c=colors[j])
        ax1.grid()
        ax1.set_xlim(-1, world_size[0]+1)
        ax1.set_ylim(-1, world_size[1]+1)
        ax1.set_xticks(list(range(world_size[0]+1)))
        ax1.set_yticks(list(range(world_size[1]+1)))
        if i < sim.time: ax1.set_xlabel('T = '+str(i),fontsize=20) 
        else: ax1.set_xlabel('T = '+str(sim.time-1),fontsize=20)
        #ax1.set_ylabel('Y',fontsize=20)
        ax1.set_title('Mult-Agent Path Planning - Simulation\nCollisions Avoided: '+str(avoid_collisions))
        ax1.grid(1)

        try: 
            ax1.scatter(collisions[i][0], collisions[i][1], 300, facecolors='none', edgecolors='r')
            if avoid_collisions: coll_string = 'COLLISION AVOIDED'
            else: coll_string = 'COLLISION'
            ax1.text(collisions[i][0]+0.5, collisions[i][1]-0.5, coll_string)
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
    #ani.save('simulation_outputs/test.gif', writer='pillow')


if __name__ == "__main__":
    # line1 = [(i,40) for i in range(30,19, -1)]
    # line2 = [(20,i) for i in range(40,61)]
    # line3 = [(i,60) for i in range(20,51)]
    # line4 = [(50,i) for i in range(60,19, -1)]; print(line4[-1])
    # line5 = [(i,20) for i in range(50,19,-1)]
    # line6 = [(i,90) for i in range(70,81)]
    # barrier_points=[line1+line2+line3+line4+line5, line6]
    # world_size = (100, 100)
    # robots_starts = [(0,0), (80, 100), (30,50), (90,90), (70,70)]
    # robots_ends = [(10,90), (10,10), (70,20), (40,30), (20,20)]
    barrier_points=[[(3,4),(2, 4), (2, 5), (2, 6), (3, 6), (4, 6),
                     (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)],
                     [(7,9),(8,9)]]
    world_size = (10, 10)
    robots_starts = [(0,0), (8, 10), (3,5), (9,9), (7,7)]
    robots_ends = [(1,9), (1,1), (7,2), (4,3), (2,1)]    
    
    main_calculation(barrier_points,
                    world_size,
                    robots_starts,
                    robots_ends)
