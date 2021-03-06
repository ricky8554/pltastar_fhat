#!/bin/bash

map=s1
obs=30
herr=""
pick=""


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

rm "tempp_$map$obs$pick$herr"
cp control "control_test$map$obs$pick$herr"
for filename in testfolder/p_star/random_obs$obs/$map*.grid; do
    echo "$filename"
    ./"control_test$map$obs$pick$herr" -file "$filename" -movement 500 < "$filename" >> "tempp_$map$obs$pick$herr"
    echo "==================================================================="
done
rm "control_test$map$obs$pick$herr"
echo "GRAPH DRAWING"
echo "random_obs${obs}_$map"
python3 graph1.py "random_obs${obs}_$map_$herr" < "tempp_$map$obs$pick$herr"
echo END
