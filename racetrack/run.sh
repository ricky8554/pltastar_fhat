#!/bin/bash

map=s1
obs=30
herr=""
pick=""
mv control control_test

for (( i=1; i <= "$#"; i++ )); do


    #both
    if [ ${!i} == "-map" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            map=${!var}
        fi
    fi

    if [ ${!i} == "-obs" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            obs=${!var}
        fi
    fi

    if [ ${!i} == "-herr" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            herr=${!var}
        fi
    fi

    if [ ${!i} == "-pick" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            pick=${!var}
        fi
    fi

    if [ ${!i} == "-help" ]; then
        echo "<-map maptype> default s0, <-obs numberofobs> default 30, <-help>"
        exit 1
    fi

done

rm "tempp$map$obs$herr$pick"
for filename in testfolder/Racetrack/random_obs$obs/$map*.grid; do
    echo "$filename"
    ./control_test -file "$filename" < "$filename" >> "tempp$map$obs$herr$pick"
    echo "==================================================================="
done
echo "GRAPH DRAWING"
echo "random_obs$obs _$map"
python3 graph.py "random_obs$obs _$map _$herr _$pick" < "tempp$map$obs$herr$pick"
echo END