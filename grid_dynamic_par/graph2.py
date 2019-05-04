#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns 
import re
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
orders=["PLRTA*","PLRTA*_FHAT","DYNAMIC_PLRTA*","DYNAMIC_PLRTA*_FHAT"]


if(len(sys.argv) > 1):
    name = sys.argv[1]


def output():

    list_look=[]
    list_algo=[]
    list_cost=[]
    list_nb = []
    for mv, m1 in movements.items():
        for lh, m2 in m1.items():
            for cfile, m3 in m2.items():
                for nobs, m4 in m3.items():
                    for algo, cost in m4.items():
                        if "PLRTA*" in m4:
                            list_look.append(lh)
                            list_algo.append(algo)
                            list_nb.append(nobs)
                            NOBS = nobs
                            list_cost.append( (cost - m3[nobs]["PLRTA*"])/m3[nobs]["PLRTA*"] *100)
        
        sns.set(font_scale=2.1)
        sns.set_style('white')
        data = pd.DataFrame(data={'lookahead': list_look, 'cost': list_cost, 'algorithm': list_algo, 'Number Of Dynamic Obstacles': list_nb})
        g = sns.catplot(x="lookahead", y="cost", hue="algorithm", col="Number Of Dynamic Obstacles",
                capsize=0.1, palette="colorblind",ci=95, height=8, aspect=1.3,linewidth=10, dodge=0.6,
                kind="point",linestyles="",hue_order=orders,markers=["D","^","*","s","h","o","P","p","1","H",'x','d'],scale = 1.8, legend=False, data=data, s=100)


        # g = sns.catplot(x="lookahead", y="cost", hue="algorithm", col="movements",
        #              palette="colorblind", height=6, aspect=.75, dodge=True,
        #             kind="box", data=data)
        g.despine(left=True)
        
        for l in g.ax.lines:
            plt.setp(l,linewidth=3)
        # plt.show()
        matchObj = re.match( r'.*_(\d+)_.*', current_file, re.M|re.I)
        plt.legend(loc='upper right',fontsize=22)
        plt.ylabel("cost(%)",fontsize=30)
        plt.xlabel("lookahead",fontsize=30)
        plt.title("Desity Of Static Obstacles "+str(matchObj.group(1)) + "%",fontsize=25)
       
        g.savefig("graph/" + name + ".pdf")

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
            
            if(strlist[3] != '1' and strlist[3] != '3' and strlist[3] != '7' and strlist[3] != "9"):
                continue
            
            matchObj = re.match( r'.*random_obs(\d+).*', current_file, re.M|re.I)

            num_dynamic_obs = int(matchObj.group(1))
            if not current_movement in movements:
                movements[current_movement] = {}
            if not current_lookahead in movements[current_movement]:
                movements[current_movement][current_lookahead] = {}
            if not current_file in movements[current_movement][current_lookahead]:
                movements[current_movement][current_lookahead][current_file] = {}
            if not num_dynamic_obs in movements[current_movement][current_lookahead][current_file]:
                movements[current_movement][current_lookahead][current_file][num_dynamic_obs] = {}
            if not current_algo in movements[current_movement][current_lookahead][current_file][num_dynamic_obs]:
                movements[current_movement][current_lookahead][current_file][num_dynamic_obs][current_algo] = current_cost
    except EOFError:
        output()
        break

