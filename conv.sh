#!/bin/bash

for idx in {1..20}
do
	./from.sh $idx
	./lidar_comp_dev
	./lidar_comp_dev a
	./to.sh $idx
done
