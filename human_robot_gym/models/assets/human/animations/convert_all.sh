#!/bin/bash
for fullfile in ./*.bvh; do
    filename=$(basename -- "$fullfile")
    filename="${filename%.*}"
    echo "Converting file " ${fullfile} " to " ${filename}".pkl"
    python ../../../../utils/convert_bvh.py $fullfile --save_path ${filename}".pkl"
done
