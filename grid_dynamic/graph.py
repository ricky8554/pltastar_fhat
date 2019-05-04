#!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns 
import sys

alg_map =	{
  "-1": "A*",  
  "0": "LSS_LRTA*",
  "1": "PLRTA*",
  "2": "LSS_LRTA*_FHAT",
  "3": "PLRTA*_FHAT",
  "4": "PLRTA*_MOD",
  "5": "PLRTA*_FHAT_MOD",
  "6": "DYNAMIC_LSS_LRTA*",
  "7": "DYNAMIC_PLRTA*",
  "8": "DYNAMIC_LSS_LRTA*_FHAT",
  "9": "DYNAMIC_PLRTA*_FHAT",
  "10": "DYNAMIC_PLRTA*_MOD",
  "11": "DYNAMIC_PLRTA*_FHAT_MOD"
}

movements = {}

current_algorithm = ""
current_movement = ""
name = ""
orders=[]
for value in alg_map.values():
    orders.append(value)
orders=["PLRTA*","PLRTA*_FHAT","PLRTA*_MOD","PLRTA*_FHAT_MOD","DYNAMIC_PLRTA*","DYNAMIC_PLRTA*_FHAT","DYNAMIC_PLRTA*_MOD","DYNAMIC_PLRTA*_FHAT_MOD"]


if(len(sys.argv) > 1):
    name = sys.argv[1] + "_"


def output():

    list_look=[]
    list_algo=[]
    list_cost=[]
    list_mv = []
    for mv, m1 in movements.items():
        for lh, m2 in m1.items():
            for cfile, m3 in m2.items():
                for algo, cost in m3.items():
                    if "PLRTA*" in m3:
                        list_look.append(lh)
                        list_algo.append(algo)
                        list_mv.append(mv)
                        list_cost.append(cost - m3["PLRTA*"])
        data = pd.DataFrame(data={'lookahead': list_look, 'cost': list_cost, 'algorithm': list_algo, 'movements': list_mv})
        g = sns.catplot(x="lookahead", y="cost", hue="algorithm", col="movements",
                capsize=0.05, palette="colorblind",ci=95, height=10, aspect=1.5,linewidth=100, dodge=0.5,
                kind="point",linestyles="",hue_order=orders,markers=["D","^","1","s","h","o","*","p","P","H",'x','d'], data=data)
        # g = sns.catplot(x="lookahead", y="cost", hue="algorithm", col="movements",
        #              palette="colorblind", height=6, aspect=.75, dodge=True,
        #             kind="box", data=data)
        g.despine(left=True)
        for l in g.ax.lines:
            plt.setp(l,linewidth=1)
        # plt.show()
        plt.ylabel("cost(%)")
        g.savefig("graph/" + name + mv + "movenemts.pdf" )

while True:
    try:
        s=input()
        if 'movements' in s:
            strlist = s.split(" ")
            current_movement = s.split(" ")[1]
            current_algo = alg_map[strlist[3]]
            current_lookahead = int(strlist[5])
            current_cost = float(strlist[7])
            current_file = strlist[9]
            if(strlist[3] == '1' or strlist[3] == '3'or strlist[3] == '7' or strlist[3] == "9"):
                continue
            if not current_movement in movements:
                movements[current_movement] = {}
            if not current_lookahead in movements[current_movement]:
                movements[current_movement][current_lookahead] = {}
            if not current_file in movements[current_movement][current_lookahead]:
                movements[current_movement][current_lookahead][current_file] = {}
            if not current_algo in movements[current_movement][current_lookahead][current_file]:
                movements[current_movement][current_lookahead][current_file][current_algo] = current_cost
    except EOFError:
        output()
        break

