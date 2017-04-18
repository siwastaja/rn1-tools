all: prog mcprog visudrive #ros_publisher

mcprog: mcprog.c
	gcc mcprog.c -o mcprog

prog: prog.c
	gcc prog.c -o prog

visudrive: visudrive.cc ../rn1-brain/comm.h
	g++ visudrive.cc -o visudrive -lsfml-graphics -lsfml-window -lsfml-system

ros_publisher: ros_publisher.cc
	g++ ros_publisher.cc -o ros_publisher
