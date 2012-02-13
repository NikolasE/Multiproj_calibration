#!/bin/bash

rm dist_no_init.txt

for i in {100..1900..200}
do
    

   #a="Welcome $i times"
    printf -v a "estimated_no_init_%04i.txt" $i
    printf -v b "gt_%04i.txt" $i

    echo -ne $i " " >> dist_no_init.txt
    python evaluate_ate.py $a $b >> dist_no_init.txt

    #echo 'python evaluate_ate.py $a $b >> dist.txt
done