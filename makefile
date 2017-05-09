all: prog mcprog visudrive udpserver lidar_comp_dev

lidar_comp_dev: lidar_comp_dev.c
	gcc -Wall lidar_comp_dev.c -o lidar_comp_dev -lm -std=c99

mcprog: mcprog.c
	gcc -Wall mcprog.c -o mcprog

prog: prog.c
	gcc -Wall prog.c -o prog

visudrive: visudrive.cc ../rn1-brain/comm.h
	g++ -Wall visudrive.cc -o visudrive -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system

udpserver: udpserver.c
	gcc -Wall udpserver.c -o udpserver

ros_publisher: ros_publisher.cc
	g++ -Wall ros_publisher.cc -o ros_publisher
