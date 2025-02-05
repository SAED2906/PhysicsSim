main:
	gcc src/engine.c -w -lSDL2 -lSDL2_image -lm -o bin/engine
	./bin/engine