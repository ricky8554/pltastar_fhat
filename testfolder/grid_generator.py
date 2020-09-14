#!/usr/bin/env python

import sys
import re
from random import randint
w = int(sys.argv[1])
h = int(sys.argv[2])
percent_of_obs = float(sys.argv[3])


# randomseed = int(sys.argv[3]) if len(sys.argv) > 3 else 2
# randomseeddynamic = int(sys.argv[3]) if len(sys.argv) > 4 else 10

print(w)
print(h)
print(0)


# for x in range(h):
#     s = ""
#     for y in range(w):
#         if x == 0 and y == 0:
#             s += "@"
#         elif x == h - 1 and y == w - 1:
#             s += "*"
#         else:
#             if(randint(0,randomseed) == 0):
#                 s += "!" if randint(0,randomseeddynamic) == 0 else "#"
#             else:
#                 s += "_"
#     s = re.sub('#!#', '#_!', s)
#     s = re.sub('^!#', '_#', s)
#     s = re.sub('#!$', '#_', s)
#     print(s)
grid = {}

dobs = w*h*percent_of_obs
while dobs > 0:
    key = (randint(0,w-1),randint(0,h-1))
    if key in grid:
        pass
    else:
        dobs = dobs - 1
        grid[key] = '#'

for x in range(h):
    s = ""
    for y in range(w):
        if( (y,x) in grid ):
            s += grid[(y,x)]
        else:
            s += "_"
    print(s)

        