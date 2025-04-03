#!/usr/bin/env python
# coding: utf-8

# In[27]:


#robot changes colour when they collide
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
UNIT = 20
coordinates = [(2,5),(3,5),(7,5),(8,5),(12,5),(13,5),(17,5),(18,5),
(2,7),(3,7),(7,7),(8,7),(12,7),(13,7),(17,7),(18,7),
(2,9),(3,9),(7,9),(8,9),(12,9),(13,9),(17,9),(18,9),
(2,14),(3,14),(7,14),(8,14),(12,14),(13,14),(17,14),(18,14),
(2,16),(3,16),(7,16),(8,16),(12,16),(13,16),(17,16),(18,16),
(2,18),(3,18),(7,18),(8,18),(12,18),(13,18),(17,18),(18,18),
(1,4),(2,4),(3,4),(4,4),(6,4),(7,4),(8,4),(9,4),(11,4),(12,4),(13,4),(14,4),(16,4),(17,4),(18,4),(19,4),
(1,6),(2,6),(3,6),(4,6),(6,6),(7,6),(8,6),(9,6),(11,6),(12,6),(13,6),(14,6),(16,6),(17,6),(18,6),(19,6),
(1,8),(2,8),(3,8),(4,8),(6,8),(7,8),(8,8),(9,8),(11,8),(12,8),(13,8),(14,8),(16,8),(17,8),(18,8),(19,8),
(1,10),(2,10),(3,10),(4,10),(6,10),(7,10),(8,10),(9,10),(11,10),(12,10),(13,10),(14,10),(16,10),(17,10),(18,10),(19,10),
(1,13),(2,13),(3,13),(4,13),(6,13),(7,13),(8,13),(9,13),(11,13),(12,13),(13,13),(14,13),(16,13),(17,13),(18,13),(19,13),
(1,15),(2,15),(3,15),(4,15),(6,15),(7,15),(8,15),(9,15),(11,15),(12,15),(13,15),(14,15),(16,15),(17,15),(18,15),(19,15),
(1,17),(2,17),(3,17),(4,17),(6,17),(7,17),(8,17),(9,17),(11,17),(12,17),(13,17),(14,17),(16,17),(17,17),(18,17),(19,17),
(1,19),(2,19),(3,19),(4,19),(6,19),(7,19),(8,19),(9,19),(11,19),(12,19),(13,19),(14,19),(16,19),(17,19),(18,19),(19,19)]

X_Block_pic, Y_Block_pic = zip(*coordinates)
X_Block = [x * UNIT for x in X_Block_pic]
Y_Block = [y * UNIT for y in Y_Block_pic]

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
            for i in range (4,11,2):
                self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+4)*UNIT,(i+1)*UNIT,fill='bisque4')
        for k in range (1,17,5):
            for i in range (13,20,2):
                self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+4)*UNIT,(i+1)*UNIT,fill='bisque4')
        for k in range (2,20,5):
            for i in range (5,10,2):
                self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+2)*UNIT,(i+1)*UNIT,fill='bisque4')
        for k in range (2,20,5):
            for i in range (14,19,2):
                self.canvas.create_rectangle(k*UNIT, i*UNIT, (k+2)*UNIT,(i+1)*UNIT,fill='bisque4')
               
        # create human being
        self.human1 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='') 
        self.human2 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='') 
        
        # create targets        
        self.target1 = self.canvas.create_rectangle(
            11*UNIT,7*UNIT,12*UNIT,8*UNIT,
            fill='light salmon')
        self.target2 = self.canvas.create_rectangle(
            4*UNIT,7*UNIT,5*UNIT,8*UNIT,
            fill='tomato')
        self.target3 = self.canvas.create_rectangle(
            9*UNIT,7*UNIT,10*UNIT,8*UNIT,
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

            # Check for collision with other robots
            robots = [self.rect1, self.rect2, self.rect3]
            for other_robot in robots:
                if other_robot != robot and self.canvas.coords(robot) == self.canvas.coords(other_robot):
                    # Collision detected, change color to bright pink
                    self.canvas.itemconfig(robot, fill='hot pink')

            self.canvas.coords(robot, x * UNIT, y * UNIT, x * UNIT + 20, y * UNIT + 20)
            step += 1
            self.after(1000, lambda: self.move_robot_step(robot, path, step, callback))
        elif callback:
            callback()
    def move_robot(self, robot, start, goal, callback=None):
        came_from, _ = self.a_star_search(start, goal)
        path = self.reconstruct_path(came_from, start, goal)
        self.move_robot_step(robot, path, callback=callback)
    def start(self):
        # 로봇 1 이동
        start_time1 = time.time()
        self.move_robot(self.rect1, (origin1[0] // UNIT, origin1[1] // UNIT), (10, 7),
                        callback=lambda: self.move_robot(self.rect1, (10, 7), (origin1[0] // UNIT, origin1[1] // UNIT),
                                                          callback=lambda: print("Robot 1: {:.2f} seconds".format(time.time() - start_time1))))
        
        # 로봇 2 이동
        start_time2 = time.time()
        self.move_robot(self.rect2, (origin2[0] // UNIT, origin2[1] // UNIT), (5, 7),
                        callback=lambda: self.move_robot(self.rect2, (5, 7), (origin2[0] // UNIT, origin2[1] // UNIT),
                                                          callback=lambda: print("Robot 2: {:.2f} seconds".format(time.time() - start_time2))))

        # 로봇 3 이동
        start_time3 = time.time()
        self.move_robot(self.rect3, (origin3[0] // UNIT, origin3[1] // UNIT), (10, 7),
                        callback=lambda: self.move_robot(self.rect3, (10, 7), (origin3[0] // UNIT, origin3[1] // UNIT),
                                                          callback=lambda: print("Robot 3: {:.2f} seconds".format(time.time() - start_time3))))



if __name__ == '__main__':
    maze = Maze()
    maze.after(1000, maze.start)  # 로봇 이동 시작
    maze.mainloop()


# In[ ]:





# In[ ]:




