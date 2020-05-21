all: sokoban

sokoban: bin/main.o bin/Sokoban.o
	g++ -std=c++11 -O3 -o sokoban bin/main.o bin/Sokoban.o -lm

bin/Sokoban.o: include/Sokoban.h src/Sokoban.cpp
	g++ -std=c++11 -O3 -o bin/Sokoban.o -c src/Sokoban.cpp

bin/main.o: include/Sokoban.h src/main.cpp
	g++ -std=c++11 -O3 -o bin/main.o -c src/main.cpp

bin/:
	mkdir -p bin

clean:
	rm -f sokoban
	rm -rf bin/
