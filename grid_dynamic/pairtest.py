#!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns 
import sys

alg_map =	{
  "0": "LSS_LRTA*",
  "1": "PLRTA*",
  "2": "LSS_LRTA*_FHAT",
  "3": "PLRTA*_FHAT",
  "4": "DYNAMIC_LSS_LRTA*",
  "5": "DYNAMIC_PLRTA*",
  "6": "DYNAMIC_LSS_LRTA*_FHAT",
  "7": "DYNAMIC_PLRTA*_FHAT"
}

movements = {}

current_algorithm = ""
current_movement = ""
name = ""

if(len(sys.argv) > 1):
    name = sys.argv[1] + "_"


def output():
    for value in movements.values():
        data = pd.DataFrame(data={'lookahead': value[0], 'cost': value[1], 'algorithm': value[2], 'movements': value[3]})
        g = sns.catplot(x="lookahead", y="cost", hue="algorithm", col="movements",
                    capsize=.2, palette="colorblind", height=6, aspect=.75,
                    kind="point", data=data)
        g.despine(left=True)
        for l in g.ax.lines:
            plt.setp(l,linewidth=1)
        # plt.show()
        g.savefig("graph/" + name + str(value[3][0]) + "movenemts.png" )

while True:
    try:
        s=input()
        if 'movements' in s:
            current_movement = s.split(" ")[1]
            if not current_movement in movements:
                movements[current_movement] = ([],[],[],[])
        elif 'algorithm' in s:
            current_algorithm = alg_map[s.split(" ")[1]]
        elif len(s.strip()) == 0:
            continue
        else:
            s = s.split(" ")
            movements[current_movement][0].append(int(s[0]))
            movements[current_movement][1].append(float(s[1]))
            movements[current_movement][2].append(current_algorithm)
            movements[current_movement][3].append(current_movement)
    except EOFError:
        output()
        break

