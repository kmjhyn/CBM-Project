#!/usr/bin/env python
# coding: utf-8

# In[7]:


#no collision what so ever huge environment. path is clear
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
MAZE_H = 21  # grid height
MAZE_W = 21 # grid width

#shelf coordinates
X_Block_pic = [1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19]
X_Block = [element * UNIT for element in X_Block_pic]
Y_Block_pic = [5,6,8,9,11,12,14,15,17,18]
Y_Block = [element * UNIT for element in Y_Block_pic]

origin1 = np.array([70, 50])
origin2 = np.array([210,50])
origin3 = np.array([350,50])

class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r','w'] #up, down, left, right, wait
        self.n_actions = len(self.action_space)
        self.title('Warehouse')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self._build_maze()
        self.robot_speed = 1  # Time taken to move through each block in seconds

   
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
        for i in range (1,16,7):
            self.canvas.create_rectangle(i*UNIT, 0, (i+5)*UNIT, 2*UNIT, fill = 'sandybrown')
        
        # create shelves
        for k in range (1,17,5):
            for i in range (5,18,3):
                self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+4)*UNIT,(i+2)*UNIT,fill='bisque4')
        # create human being
        self.human1 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='') 
        self.human2 = self.canvas.create_rectangle(0*UNIT, 1*UNIT, 1*UNIT, 2*UNIT, fill='') 
        
        # create targets        
        self.target1 = self.canvas.create_rectangle(
            19*UNIT,10*UNIT,20*UNIT,11*UNIT,
            fill='light salmon')
        self.target2 = self.canvas.create_rectangle(
            2*UNIT,10*UNIT,3*UNIT,11*UNIT,
            fill='tomato')
        self.target3 = self.canvas.create_rectangle(
            8*UNIT,10*UNIT,9*UNIT,11*UNIT,
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

    def move_robot_step(self, robot, path, step=0, callback=None, start_time=None):
        if step < len(path):
            x, y = path[step]
            self.canvas.coords(robot, x * UNIT, y * UNIT, x * UNIT + 20, y * UNIT + 20)
            step += 1
            self.after(int(self.robot_speed * 1000), lambda: self.move_robot_step(robot, path, step, callback, start_time))
        elif callback:
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"The time taken for the robot to reach its goal and come back to its original position: {elapsed_time:.2f} seconds")
            callback()

    def move_robot(self, robot, start, goal, callback=None):
        start_time = time.time()
        came_from, _ = self.a_star_search(start, goal)
        path = self.reconstruct_path(came_from, start, goal)
        self.move_robot_step(robot, path, callback=callback, start_time=start_time)

    def start(self):
        # Define callback functions for each robot
        def robot1_callback():
            self.move_robot(self.rect1, (origin1[0] // UNIT, origin1[1] // UNIT), (2, 10),
                            callback=lambda: self.move_robot(self.rect1, (2, 10), (origin1[0] // UNIT, origin1[1] // UNIT)))

        def robot2_callback():
            self.move_robot(self.rect2, (origin2[0] // UNIT, origin2[1] // UNIT), (8, 10),
                            callback=lambda: self.move_robot(self.rect2, (8, 10), (origin2[0] // UNIT, origin2[1] // UNIT)))

        def robot3_callback():
            self.move_robot(self.rect3, (origin3[0] // UNIT, origin3[1] // UNIT), (19, 10),
                            callback=lambda: self.move_robot(self.rect3, (19, 10), (origin3[0] // UNIT, origin3[1] // UNIT)))

        # Start the robots
        self.move_robot(self.rect1, (origin1[0] // UNIT, origin1[1] // UNIT), (2, 10), callback=robot1_callback)
        self.move_robot(self.rect2, (origin2[0] // UNIT, origin2[1] // UNIT), (8, 10), callback=robot2_callback)
        self.move_robot(self.rect3, (origin3[0] // UNIT, origin3[1] // UNIT), (19, 10), callback=robot3_callback)

if __name__ == '__main__':
    maze = Maze()
    maze.after(1000, maze.start)  # Robots start moving after 1 second
    maze.mainloop()


# In[ ]:




