#!/bin/bash

brainfw="/home/hrst/rn1-tools/main.bin"
motconfw="/home/hrst/rn1-tools/motcon.bin"
brainfwtmp="/home/hrst/rn1-tools/main_tmp.bin"
motconfwtmp="/home/hrst/rn1-tools/motcon_tmp.bin"
prog="/home/hrst/rn1-tools/prog"
mcprog="/home/hrst/rn1-tools/mcprog"
udpser="/home/hrst/rn1-tools/udpserver"

uart="/dev/serial0"

while true
do
	if [ -f "$brainfwtmp" ]
	then
		rm $brainfwtmp
	fi

	if [ -f "$motconfwtmp" ]
	then
		rm $motconfwtmp
	fi

	if [ -f "$brainfw" ]
	then
		echo "Reflashing main cpu..."
		mv $brainfw $brainfwtmp
		$prog $uart $brainfwtmp s
		rm $brainfwtmp
		sleep 3
	fi

	if [ -f "$motconfw" ]
	then
		echo "Reflashing motor controller cpu 3..."
		mv $motconfw $motconfwtmp
		$mcprog $uart $motconfwtmp 3
		sleep 4
		echo "Reflashing motor controller cpu 4..."
		$mcprog $uart $motconfwtmp 4
		rm $motconfwtmp
		sleep 3
	fi

	echo "Starting udpserver..."
	$udpser $uart 22334
	sleep 1
done
