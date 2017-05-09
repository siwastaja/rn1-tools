#!/bin/bash

for idx in {1..20}
do
	echo "Image $idx"
	./from.sh $idx
	./lidar_comp_dev
	./lidar_comp_dev a
	./to.sh $idx
done

for idx in {1..20}
do
	diff -q ${idx}lidar_after_out ${idx}lidar_after_out_alt
done
