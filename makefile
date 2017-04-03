all: prog visudrive

prog: prog.c
	gcc prog.c -o prog

visudrive: visudrive.cc
	g++ visudrive.cc -o visudrive -lsfml-graphics -lsfml-window -lsfml-system
