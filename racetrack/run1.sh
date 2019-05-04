#!/bin/bash
rm 200_200temp
for filename in testfolder/200_200*.grid; do
    echo "$filename"
    ./control < "$filename" >> 200_200temp
    echo "==================================================================="
done
echo "GRAPH DRAWING"
python3 graph.py 200_200 < 200_200temp
echo END