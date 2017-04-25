#!/bin/bash

brainfw="/home/hrst/rn1-tools/main.bin"
motconfw="/home/hrst/rn1-tools/motcon.bin"
uart="/dev/serial0"

while true
do
	if [ -f "$brainfw" ]
	then
		echo "Reflashing main cpu..."
		./prog $uart $brainfw s
		rm $brainfw
	fi

	if [ -f "$motconfw" ]
	then
		echo "Reflashing motor controller cpu 3..."
		./mcprog $uart $motconfw 3
		echo "Reflashing motor controller cpu 4..."
		./mcprog $uart $motconfw 4
		rm $motconfw
	fi

	echo "Starting udpserver..."
	./udpserver $uart 22334
	sleep 1
done
