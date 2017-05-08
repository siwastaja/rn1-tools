all: prog mcprog visudrive udpserver lidar_comp_dev

lidar_comp_dev: lidar_comp_dev.c
	gcc lidar_comp_dev.c -o lidar_comp_dev -lm -std=c99

mcprog: mcprog.c
	gcc mcprog.c -o mcprog

prog: prog.c
	gcc prog.c -o prog

visudrive: visudrive.cc ../rn1-brain/comm.h
	g++ visudrive.cc -o visudrive -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system

udpserver: udpserver.c
	gcc udpserver.c -o udpserver

ros_publisher: ros_publisher.cc
	g++ ros_publisher.cc -o ros_publisher
