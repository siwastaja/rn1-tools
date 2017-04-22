#!/bin/bash

brainfw="main.bin"
motconfw="motcon.bin"

while true
do
	if [ -f "$brainfw" ]
	then
		echo "Reflashing main cpu..."
		./prog /dev/serial1 $brainfw h
		rm $brainfw
	fi

	if [ -f "$motconfw" ]
	then
		echo "Reflashing motor controller cpu 3..."
		./mcprog /dev/serial1 $motconfw 3
		echo "Reflashing motor controller cpu 4..."
		./mcprog /dev/serial1 $motconfw 4
		rm $motconfw
	fi

	echo "Starting udpserver..."
	./udpserver /dev/serial1 22334
	sleep 1
done
