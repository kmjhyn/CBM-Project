#!/usr/bin/env python
# coding: utf-8

# In[20]:


# -*- coding: utf-8 -*-



import numpy as np
import time
import sys
import heapq


if sys.version_info.major == 2:
    import tkinter as tk
else:
    import tkinter as tk

class PriorityQueue:
        def __init__(self):
            self.elements = []

        def empty(self):
            return len(self.elements) == 0

        def put(self, item, priority):
            heapq.heappush(self.elements, (priority, item))

        def get(self):
            return heapq.heappop(self.elements)[1]

UNIT = 20   # pixels
MAZE_H = 11  # grid height
MAZE_W = 11 # grid width

#shelf coordinatesç
coordinates = [(2,5),(3,5),(7,5),(8,5),(2,7),(3,7),(7,7),(8,7),
               (1,4),(2,4),(3,4),(4,4),(6,4),(7,4),(8,4),(9,4),(1,6),(2,6),(3,6),(4,6),(6,6),(7,6),(8,6),(9,6),
               (1,8),(2,8),(3,8),(4,8),(6,8),(7,8),(8,8),(9,8)]
X_Block_pic, Y_Block_pic = zip(*coordinates)
X_Block = [x * UNIT for x in X_Block_pic]
Y_Block = [y * UNIT for y in Y_Block_pic]

origin1 = np.array([70, 50])
origin2 = np.array([110, 50])
origin3 = np.array([150, 50])

class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r','w'] #up, down, left, right, wait
        self.n_actions = len(self.action_space)
        self.title('Warehouse')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_W * UNIT))
        self._build_maze()

   
    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg='oldlace',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # create grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_W * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)
        
        # create operation desks
        self.canvas.create_rectangle(3*UNIT, 0*UNIT, 8*UNIT, 2*UNIT, fill='sandybrown')

        # create shelves
        shelf_coordinates = [(2,5),(3,5),(7,5),(8,5),(2,7),(3,7),(7,7),(8,7),
               (1,4),(2,4),(3,4),(4,4),(6,4),(7,4),(8,4),(9,4),(1,6),(2,6),(3,6),(4,6),(6,6),(7,6),(8,6),(9,6),
               (1,8),(2,8),(3,8),(4,8),(6,8),(7,8),(8,8),(9,8)]
        for coord in shelf_coordinates:
            k, i = coord
            self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+1)*UNIT, (i+1)*UNIT, fill='bisque4')
    # create human being
        self.human1 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='') 
        self.human2 = self.canvas.create_rectangle(0*UNIT, 1*UNIT, 1*UNIT, 2*UNIT, fill='') 
        
        # create targets        
        self.target1 = self.canvas.create_rectangle(
            4*UNIT,5*UNIT,5*UNIT,6*UNIT,
            fill='light salmon')
        self.target2 = self.canvas.create_rectangle(
            6*UNIT,5*UNIT,7*UNIT,6*UNIT,
            fill='tomato')
        self.target3 = self.canvas.create_rectangle(
            4*UNIT,7*UNIT,5*UNIT,8*UNIT,
            fill='orangered')
        # define starting points       
        self.org1 = self.canvas.create_rectangle(
            origin1[0] - 10, origin1[1] - 10,
            origin1[0] + 10, origin1[1] + 10)  
        self.org2 = self.canvas.create_rectangle(
            origin2[0] - 10, origin2[1] - 10,
            origin2[0] + 10, origin2[1] + 10) 
        self.org3 = self.canvas.create_rectangle(
            origin3[0] - 10, origin3[1] - 10,
            origin3[0] + 10, origin3[1] + 10)         

        #create robot1
        self.rect1 = self.canvas.create_rectangle(
            origin1[0] - 10, origin1[1] - 10,
            origin1[0] + 10, origin1[1] + 10,
            fill='SkyBlue1')
        #create robot2
        self.rect2 = self.canvas.create_rectangle(
            origin2[0] - 10, origin2[1] - 10,
            origin2[0] + 10, origin2[1] + 10,
            fill='SteelBlue2')
        #create robot3
        self.rect3 = self.canvas.create_rectangle(
            origin3[0] - 10, origin3[1] - 10,
            origin3[0] + 10, origin3[1] + 10,
            fill='RoyalBlue1' )
        # pack all
        self.canvas.pack()
        
    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < MAZE_W and 0 <= y < MAZE_H

    def passable(self, id):
        return id not in self.obstacles()

    def obstacles(self):
        return {(x // UNIT, y // UNIT) for x in X_Block for y in Y_Block}

    def a_star_search(self, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + 1  # 모든 이동 비용은 1로 가정
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        return came_from, cost_so_far

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)  # optional
        path.reverse()  # optional
        return path

    def move_robot_step(self, robot, path, step=0, callback=None):
        if step < len(path):
            x, y = path[step]
            self.canvas.coords(robot, x * UNIT, y * UNIT, x * UNIT + 20, y * UNIT + 20)
            step += 1
            # Adjust the delay to 1000 milliseconds (1 second)
            self.after(1000, lambda: self.move_robot_step(robot, path, step, callback))
        elif callback:
            callback()
    def move_robot(self, robot, start, goal, callback=None):
        came_from, _ = self.a_star_search(start, goal)
        path = self.reconstruct_path(came_from, start, goal)
        self.move_robot_step(robot, path, callback=callback)
    def start(self):
        # Record the start time
        self.start_time = datetime.now()

        # Robot 1 movement
        self.move_robot(
            self.rect1, (origin1[0] // UNIT, origin1[1] // UNIT), (5, 5),
            callback=lambda: self.move_robot(
                self.rect1, (5, 5), (origin1[0] // UNIT, origin1[1] // UNIT),
                callback=self.finish_task
            )
        )

        # Robot 2 movement
        self.move_robot(
            self.rect2, (origin2[0] // UNIT, origin2[1] // UNIT), (5, 5),
            callback=lambda: self.move_robot(
                self.rect2, (5, 5), (origin2[0] // UNIT, origin2[1] // UNIT),
                callback=self.finish_task
            )
        )

        # Robot 3 movement
        self.move_robot(
            self.rect3, (origin3[0] // UNIT, origin3[1] // UNIT), (5, 7),
            callback=lambda: self.move_robot(
                self.rect3, (5, 7), (origin3[0] // UNIT, origin3[1] // UNIT),
                callback=self.finish_task
            )
        )

    def finish_task(self):
        # Calculate the total time taken
        end_time = datetime.now()
        total_time = end_time - self.start_time
        print(f"Total time taken: {total_time}")



if __name__ == '__main__':
    maze = Maze()
    maze.after(1000, maze.start)  # 로봇 이동 시작
    maze.mainloop()


# In[ ]:




