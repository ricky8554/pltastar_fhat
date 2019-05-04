#!/bin/bash
for filename in file/*; do
    file=$(basename $filename)
    echo "$file"
    python3 domaing.py -m $file -s 500 -o 50 < $filename
    echo "==================================================================="
done