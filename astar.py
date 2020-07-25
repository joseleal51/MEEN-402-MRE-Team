from __future__ import print_function
import matplotlib.pyplot as plt
from math import sqrt

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
        self.time = range(self.longest_route_len)
        self.num_robots = len(routes)

        #self.priorities = 

        #print('yoyo', self.longest_route_len)

        """ Transpose paths to time - becasue the routes input is a list of list where the 
            first list is the route of one robot. Example: [[robot 1 route in tuple's of points],
            [robot 2 route in tuple's of points]]
            After being transposted the first list in the list is the first point of each robot
            Example: [[robo1 point 1, robo2 point 1], [robo1 point 2, robo2 point 2]]
        """

        self.routes_time = [[(0,0)]*self.num_robots for t in self.time]
        for t in self.time:
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
        for i in range(len(self.routes_time)): print(self.routes_time[i], 'T = ',i)
        for t in self.time:
            for robo in range(self.num_robots):
                pass


        # Find collisons
        collisions = {}
        c = []
        for i, locations in enumerate(self.routes_time):
            a = [locations.count(j) for j in locations]
            print('T= ',i, 'Count of repeated points (2 means collision)', a)
        
            if max([locations.count(j) for j in locations]) > 1:
                print(max([locations.count(j) for j in locations]))
                print('COLLISION at t=', i)


        #for path in routes.values():
            #for way_point in path:
                #print(way_point)

        # Create robot "objects???" good idea to make another class???? Just hold info in a dictionary for now


    def find_collisions(self):
        pass          


def main_calculation(
    barrier_points=[[(3,4),(2, 4), (2, 5), (2, 6), (3, 6), (4, 6),
                     (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)],
                     [(7,9),(8,9)]],
    world_size = (10, 10),
    robots_starts = [(0,0), (8, 10), (3,5), (9,9), (7,7)],
    robots_ends = [(8,4), (1,1), (7,3), (3,3), (2,2)]):
    
    num_robots = len(robots_starts)
    graph = AStarGraph(world_size, barrier_points)
    results = []#{v:None for v in range(num_robots)}
    costs = []

    for i in range(num_robots):
        my_tuple = AStarSearch.solve(robots_starts[i], robots_ends[i], graph)
        #results[i] = my_tuple[0]
        results.append(my_tuple[0])
        costs.append(my_tuple[1])
    print('paths:')
    for r in results: print(r, len(r))
    #result, cost = AStarSearch.solve(robot_star, robot_end, graph)

    #for i in range(num_robots): print("\nroute "+str(i+1)+": ", results[i], "\ncost", costs[i])

    sim = ScheduleRobots(results)

    """     for result in results:
        plt.plot([x[0] for x in result], [x[1] for x in result])
    for barrier in graph.barriers:
        #print('\nBarrier: ', barrier)
        x = [v[0] for v in barrier]
        y = [v[1] for v in barrier]
        plt.plot(x, y, 'k')

    for i in range(num_robots):
        plt.scatter(robots_starts[i][0],robots_starts[i][1],s=40,c='r')
        plt.scatter(robots_ends[i][0],robots_ends[i][1],s=40,c='g')
        plt.text(robots_starts[i][0],robots_starts[i][1],"Start "+str(i+1))
        plt.text(robots_ends[i][0],robots_ends[i][1],"End"+str(i+1))
    plt.xlim(-1, world_size[0]+1)
    plt.ylim(-1, world_size[1]+1)
    plt.xticks(list(range(world_size[0]+1)))
    plt.yticks(list(range(world_size[1]+1)))
    #plt.title('G_diag:'+str())
    plt.grid()
    plt.show()#block=False) """

    
    # animation?
    #Writer = animation.writers['pillow']#'ffmpeg']
    #writer = Writer(fps=1, metadata=dict(artist='Mike Ashley'), bitrate=1800)

    fig = plt.figure(figsize=(10,6))
    #ax = plt.axes(xlim=(-50, 50), ylim=(-50, 50)) 
    ax1 = fig.add_subplot(1,1,1)
    data = sim.routes_time
    #print(['Robo1'*num_robots])
    colors = {0:'k', 1:'r', 2:'g', 3:'b', 4:'c', 5:'y', 6:'g'}
 
    def animate(i):
        ax1.clear()
        for barrier in graph.barriers:
        #print('\nBarrier: ', barrier)
            x = [v[0] for v in barrier]
            y = [v[1] for v in barrier]
            ax1.plot(x, y, 'k')
        points = data[i]
        for j, robo in enumerate(points):
            #print('robo: ', robo, 'j: ', j)
            ax1.scatter(robo[0], robo[1], s=100, c=colors[j])
        ax1.grid()
        #ax1.scatter(i,i,s=40,c='g')
        ax1.set_xlim(-1, world_size[0]+1)
        ax1.set_ylim(-1, world_size[1]+1)
        ax1.set_xticks(list(range(world_size[0]+1)))
        ax1.set_yticks(list(range(world_size[1]+1)))
        ax1.set_xlabel('X',fontsize=20)
        ax1.set_ylabel('Y',fontsize=20)
        ax1.set_title('Mult-Agent Path Planning - Simulation')
        ax1.grid(1)
        for j, result in enumerate(results):
            ax1.plot([x[0] for x in result], [x[1] for x in result], c=colors[j] )
        for i in range(num_robots):
            ax1.scatter(robots_starts[i][0],robots_starts[i][1],s=40,c=colors[i])
            ax1.scatter(robots_ends[i][0],robots_ends[i][1],s=40,c=colors[i])
            ax1.text(robots_starts[i][0],robots_starts[i][1],"Start "+str(i+1))
            ax1.text(robots_ends[i][0],robots_ends[i][1],"End"+str(i+1))
    plt.xlim(-1, world_size[0]+1)
    plt.ylim(-1, world_size[1]+1)
    plt.xticks(list(range(world_size[0]+1)))
    plt.yticks(list(range(world_size[1]+1)))
    plt.xlabel('X',fontsize=20)
    plt.ylabel('Y',fontsize=20)
    plt.title('Mult-Agent Path Planning - Simulation')
    plt.grid()   

    ani = animation.FuncAnimation(fig, animate, interval=400, frames=13, repeat=True)
    plt.show()
    plt.xlim(-1, world_size[0]+1)
    plt.ylim(-1, world_size[1]+1)
    plt.xticks(list(range(world_size[0]+1)))
    plt.yticks(list(range(world_size[1]+1)))
    plt.xlabel('X',fontsize=20)
    plt.ylabel('Y',fontsize=20)
    plt.title('Mult-Agent Path Planning - Simulation')
    plt.grid()
    ani.save('MRE.gif', writer='pillow')#writer)
    plt.show()

# animate graph - make time := distance (need to define a min distance/time that robots can get to eachother (collison avoidance))


if __name__ == "__main__":
    main_calculation()

