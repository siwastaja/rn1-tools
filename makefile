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

tofdev: tofdev.cc
	g++ -fpermissive -D_GLIBCXX_USE_CXX11_ABI=0 -Wall -c -o rn1client.o tofdev.cc
	g++ -D_GLIBCXX_USE_CXX11_ABI=0 -o tofdev rn1client.o -L/usr/local/lib -lm -lsfml-graphics -lsfml-window -lsfml-system

tof_table_gen: tof_table_gen.c
	gcc -Wall tof_table_gen.c -o tof_table_gen -lm -std=c99
