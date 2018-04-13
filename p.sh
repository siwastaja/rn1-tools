#!/bin/bash

# This script reflashes main cpu and/or two motor controller CPUs
# if the firmware files exist. The files are renamed to prevent
# unnecessary flashing again.
# If no files are present, nothing happens. For example, if you want,
# you could run this script at every boot..



# Fill in the motor controller indexes (1 to 4) in use
# Order doesn't matter - both are reflashed
motcona=1
motconb=4

tooldir="~/rn1-tools"

lockfile="~/rn1-host/rn1host.lock"

logfile="${tooldir}/program_log.txt"



# Actual firmware filenames expected
brainfw="${tooldir}/main.bin"
motconfw="${tooldir}/motcon.bin"


brainfwtmp="${tooldir}/main_tmp.bin"
motconfwtmp="${tooldir}/motcon_tmp.bin"

# Programming executables
prog="${tooldir}/prog"
mcprog="${tooldir}/mcprog"

uart="/dev/serial0"

	if [ -f "$lockfile" ]
	then
		echo "Error: rn1host lockfile in place, maybe it's running? ($lockfile)"
		echo "Error: rn1host lockfile in place, maybe it's running? ($lockfile)" > $logfile
		exit 1
	fi

	echo "Running programmer" > $logfile
	echo "Running programmer"
	date >> $logfile

	if [ -f "$brainfwtmp" ]
	then
		rm $brainfwtmp
	fi

	if [ -f "$brainfw" ]
	then
		echo "Reflashing main cpu..."
		echo "Reflashing main cpu..." >> $logfile
		mv $brainfw $brainfwtmp
		$prog $uart $brainfwtmp s >> $logfile
		rm $brainfwtmp
		sleep 3
	fi

	if [ -f "$motconfwtmp" ]
	then
		rm $motconfwtmp
	fi

	if [ -f "$motconfw" ]
	then
		echo "Reflashing motor controller cpu ${motcona}..."
		echo "Reflashing motor controller cpu ${motcona}..." >> $logfile
		mv $motconfw $motconfwtmp
		$mcprog $uart $motconfwtmp ${motcona} >> $logfile
		sleep 4
		echo "Reflashing motor controller cpu ${motconb}..."
		echo "Reflashing motor controller cpu ${motconb}..." >> $logfile
		$mcprog $uart $motconfwtmp ${motconb} >> $logfile
		rm $motconfwtmp
		sleep 3
	fi
