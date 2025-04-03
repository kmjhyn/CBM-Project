#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 20:00:07 2020

@author: jingci
"""

import numpy as np
import time
import sys

if sys.version_info.major == 2:
    import tkinter as tk
else:
    import tkinter as tk

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

    def resetRobot(self):
        self.update()
        time.sleep(0.01)
        self.canvas.delete(self.rect1)
        self.canvas.delete(self.rect2)
        self.canvas.delete(self.rect3)
        self.rect1 = self.canvas.create_rectangle(
            origin1[0] - 10, origin1[1] - 10,
            origin1[0] + 10, origin1[1] + 10,
            fill='SkyBlue1')
        self.rect2 = self.canvas.create_rectangle(
            origin2[0] - 10, origin2[1] - 10,
            origin2[0] + 10, origin2[1] + 10,
            fill='SteelBlue2')   
        self.rect3 = self.canvas.create_rectangle(
            origin3[0] - 10, origin3[1] - 10,
            origin3[0] + 10, origin3[1] + 10,
            fill='RoyalBlue1' )
        return self.canvas.coords(self.rect1), self.canvas.coords(self.rect2), self.canvas.coords(self.rect3)
    
    def resetHuman(self):
        self.update()
        time.sleep(0.2)
        self.canvas.delete(self.human1)
        self.canvas.delete(self.human2)
        self.human1 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='')
        self.human2 = self.canvas.create_rectangle(0*UNIT, 0*UNIT, 1*UNIT, 1*UNIT, fill='') 
        return self.canvas.coords(self.human1), self.canvas.coords(self.human2)
    
    def humanStep1(self, action): 
        s = self.canvas.coords(self.human1)
        base_action = moveAgent(s, action)
        self.canvas.move(self.human1, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.human1)  # next state
        return s_
    
    def humanStep2(self, action): 
        s = self.canvas.coords(self.human2)
        base_action = moveAgent(s, action)
        self.canvas.move(self.human2, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.human2)  # next state
        return s_
    
    
    def returnStep1(self, action): 
        s = self.canvas.coords(self.rect1)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect1, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect1)  # next state
      
        # reward function 
        
        if s_ == self.canvas.coords(self.org1):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'   
        elif s_ == self.canvas.coords(self.target2) or s_ == self.canvas.coords(self.target3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.org2) or s_ == self.canvas.coords(self.org3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or s_[1] < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
              
        return s_, reward, done

    def returnStep2(self, action): 
        s = self.canvas.coords(self.rect2)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect2, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect2)  # next state
      
        # reward function 
        
        if s_ == self.canvas.coords(self.org2):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'   
        elif s_ == self.canvas.coords(self.target1) or s_ == self.canvas.coords(self.target3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.org1) or s_ == self.canvas.coords(self.org3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or s_[1] < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
              
        return s_, reward, done
    
    def returnStep3(self, action): 
        s = self.canvas.coords(self.rect3)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect3, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect3)  # next state
      
        # reward function 
        
        if s_ == self.canvas.coords(self.org3):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'   
        elif s_ == self.canvas.coords(self.target1) or s_ == self.canvas.coords(self.target2):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.org1) or s_ == self.canvas.coords(self.org2):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or s_[1] < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
              
        return s_, reward, done    
    
    def step1(self, action, obstacle=None):
        s = self.canvas.coords(self.rect1)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect1, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect1)  # next state
      
        # reward function 
        
        if s_ == self.canvas.coords(self.target1):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'   
        elif s_ == self.canvas.coords(self.target2) or s_ == self.canvas.coords(self.target3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or s_[1] < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
        
        if obstacle != None:
            if s_ == obstacle:
                reward = -50
                done = 'hit'
                s_ = 'terminal'
        
        return s_, reward, done
    
    def step2(self, action, obstacle=None):
        s = self.canvas.coords(self.rect2)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect2, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect2)  # next state
      
        # reward function  
        if s_ == self.canvas.coords(self.target2):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.target1) or s_ == self.canvas.coords(self.target3):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or int(s_[1]) < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
                
        if obstacle != None:
            if s_ == obstacle:
                reward = -50
                done = 'hit'
                s_ = 'terminal'

        return s_, reward, done      

    def step3(self, action, obstacle=None):
        s = self.canvas.coords(self.rect3)
        base_action = moveAgent(s, action)
        self.canvas.move(self.rect3, base_action[0], base_action[1])  # move agent
        s_ = self.canvas.coords(self.rect3)  # next state
      
        # reward function

        if s_ == self.canvas.coords(self.target3):
            reward = 50
            done = 'arrive'
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.target1) or s_ == self.canvas.coords(self.target2):
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif s_[0] == 0 or s_[1] < 40 or s_[2] >= MAZE_H * UNIT or s_[3] >= MAZE_W * UNIT:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        elif int(s_[0]) in X_Block and int(s_[1]) in Y_Block:
            reward = -50
            done = 'hit'
            s_ = 'terminal'
        else:
            reward = 0
            done = 'nothing'
       
        if obstacle != None:
            if s_ == obstacle:
                reward = -50
                done = 'hit'
                s_ = 'terminal'
          
        return s_, reward, done

    def render(self):
        time.sleep(0.01)
        self.update()

    
def moveAgent(s, action):
    base_action = np.array([0, 0])
    if action == 0:   # up
        if s[1] > UNIT:
            base_action[1] -= UNIT
    elif action == 1:   # down
        if s[1] < (MAZE_H - 1) * UNIT:
            base_action[1] += UNIT
    elif action == 2:   # right
        if s[0] < (MAZE_W - 1) * UNIT:
            base_action[0] += UNIT
    elif action == 3:   # left
        if s[0] > UNIT:
            base_action[0] -= UNIT
    elif action == 4:   # wait
        base_action = np.array([0, 0])
    return base_action

def update():
    for t in range(10):
        s1, s2 = env.resetRobot()
        while True:
            env.render()
                      
            s1,r1, done1 = env.step1(2)
            s2,r2,done2 = env.step2(1)
            
            if done1 == 'hit' and done2 == 'hit':
                break
            elif done1 == 'hit' and done2 == 'nothing':
                break
            elif done1 == 'arrive' and done2 == 'arrive':
                break
            elif s1 == s2:
                break

if __name__ == '__main__':
    env = Maze()
    env.after(2000, update)
    env.mainloop()