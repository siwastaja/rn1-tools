#!/bin/bash

brainfw="/home/hrst/rn1-tools/main.bin"
motconfw="/home/hrst/rn1-tools/motcon.bin"
prog="/home/hrst/rn1-tools/prog"
mcprog="/home/hrst/rn1-tools/mcprog"
udpser="/home/hrst/rn1-tools/udpserver"

uart="/dev/serial0"

while true
do
	if [ -f "$brainfw" ]
	then
		echo "Reflashing main cpu..."
		$prog $uart $brainfw s
		sleep 3
		rm $brainfw
	fi

	if [ -f "$motconfw" ]
	then
		echo "Reflashing motor controller cpu 3..."
		$mcprog $uart $motconfw 3
		sleep 3
		echo "Reflashing motor controller cpu 4..."
		$mcprog $uart $motconfw 4
		sleep 3
		rm $motconfw
	fi

	echo "Starting udpserver..."
	$udpser $uart 22334
	sleep 1
done
