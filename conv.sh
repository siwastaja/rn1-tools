#!/bin/bash

for idx in {1..10}
do
	./from.sh $idx
	./lidar_comp_dev
	./to.sh $idx
done
