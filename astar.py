from __future__ import print_function
import matplotlib.pyplot as plt


class AStarGraph(object):
    # Define a class board like grid with two barriers

    def __init__(self, world_size, barrier_points):
        self.x_size = world_size[0]
        self.y_size = world_size[1]
        self.barriers = barrier_points
        # print("type: ", type(self.barriers))

    def heuristic(self, start, goal):
        # Use Chebyshev distance heuristic if we can move one square either
        # adjacent or diagonal
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def get_vertex_neighbours(self, pos):
        n = []
        # Moves allow link a chess king
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > self.x_size or y2 < 0 or y2 > self.y_size:
                continue
            n.append((x2, y2))
        return n

    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b in barrier:
                return 100  # Extremely high cost to enter barrier squares
        return 1  # Normal movement cost


class AStarSearch:
    """Class of functions for A* search algorithm"""
    def __init__(cls):
        pass

    @classmethod
    def solve(cls, start, end, graph):
        G={}  # Actual movement cost to each position from the start position
        F={}  # Estimated movement cost of start to end going via this position

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

        raise RuntimeError("A* failed to find a solution")

if __name__ == "__main__":
    barrier_points=[(2, 4), (2, 5), (2, 6), (3, 6), (4, 6),
                     (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)]
    barrier_points = [(5, 6), (5, 5), (5, 4), (5,3)]
    world_size = (7,7)
    graph = AStarGraph(world_size, barrier_points)
    robot_start, robot_end = (0, 0), (7, 7)
    result, cost = AStarSearch.solve(robot_start, robot_end, graph)
    print ("route", result)
    print ("cost", cost)
    plt.plot([v[0] for v in result], [v[1] for v in result])
    for barrier in graph.barriers:
        print (type(v[0]))
        plt.plot([v[0] for v in barrier], [v[1] for v in barrier])
    plt.xlim(-1, world_size[0]+1)
    plt.ylim(-1, world_size[1]+1)
    plt.grid()
    plt.show()
