#!/usr/bin/env python

import sys
import re
import random
from random import randint
import math
import os

w = int(input())
h = int(input())
obs = int(input())
obs = 30
total_step = 500
obs_map = {}
d_obs_map = {}
start_goal_map = {}
dynamic_list = []


def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


def check_valid(loc, parm):

    angle, speed, steps = parm
    cx, cy = loc
    fx = cx + math.cos(angle) * speed * steps
    fy = cy + math.sin(angle) * speed * steps
    interval = distance((cx, cy), (fx, fy))/0.1
    if(interval != 0):
        diffx = (fx - cx) / interval
        diffy = (fy - cy) / interval
    else:
        return (cx, cy), parm

    interval = int(interval)
    cox = cx
    coy = cy
    for i in range(1, interval+1):
        cox = cx + diffx * i
        coy = cy + diffy * i
        compare = (int(cox), int(coy))
        # sys.stderr.write( str( compare) + '\n')
        if (compare in obs_map) or cox < 1 or cox >= w - 1 or coy < 1 or coy >= h - 1:
            return False
    return (cox, coy), parm


def get_parm():
    return random.uniform(0, 2*math.pi), round(random.uniform(0.1, 3), 1), randint(1, 20)


def dynamic_generator():
    global total_step
    accsteps = 0
    dlist = []
    loc = initial_loc = (randint(0, w-1), randint(0, h-1))
    while initial_loc in obs_map:
        initial_loc = (randint(0, w-1), randint(0, h-1))
    # sys.stderr.write(str(loc)+" FIRST \n")
    d_obs_map[initial_loc] = True
    while(total_step > accsteps):
        parm = check_valid(loc, get_parm())
        if parm != False:
            # sys.stderr.write(str(loc)+" IN \n")
            loc, par = parm
            dlist.append(par)
            accsteps += par[2]

    s = str(initial_loc[0]) + " " + str(initial_loc[1]) + " " + str(len(dlist))
    for d in dlist:
        s += " " + str(d[0]) + " " + str(d[1]) + " " + str(d[2])
    return s


def find_pair():
    while True:
        start = (randint(0, w-1), randint(0, h-1))
        goal = (randint(0, w-1), randint(0, h-1))
        while start in obs_map or start in d_obs_map:
            start = (randint(0, w-1), randint(0, h-1))
        while goal in obs_map or goal == start:
            goal = (randint(0, w-1), randint(0, h-1))
        if (not ((start, goal) in start_goal_map) ) and distance(start,goal) > w /2 :
            break
    start_goal_map[(start, goal)] = True
    return start, goal


def output():
    print_initial = str(w) + "\n" + str(h)
    # sys.stderr.write(str(obs_map))
    for _ in range(obs):
        sys.stderr.write("++++++++++++++++++++++\n")
        dynamic_list.append(dynamic_generator())

    for i in range(36):
        start,goal = find_pair()
        for obs_num in range(1,obs+1):
            print_obs = print_initial + "\n" + str(obs_num)
            for j in range(obs_num):
                print_obs += "\n" +dynamic_list[j]
            for x in range(h):
                s = ""
                for y in range(w):
                    if start == (y, h - x - 1):
                        s += "@"
                    elif goal == (y, h - x - 1):
                        s += "*"
                    elif (y, h - x - 1) in obs_map:
                        s += "#"
                    else:
                        s += "_"
                print_obs += "\n" + s 
            folder = "random_obs" + str(obs_num)
            if not os.path.exists(folder):
                os.makedirs(folder)
            filename = folder+ "/s1_" + str(i) + ".grid"
            f = open(filename, "w")
            f.write(print_obs)


while True:
    try:
        for i in range(h):
            s = input()
            for j in range(w):
                if s[j] == '#':
                    obs_map[(j, h - 1 - i)] = True
    except EOFError:
        output()
        break
