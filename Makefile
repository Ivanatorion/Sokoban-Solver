CC=g++

ifeq ($(OS),Windows_NT)
all: sokoban.exe

sokoban.exe: bin/main.o bin/Sokoban.o bin/BoxPositionSet.o bin/StateSet.o bin/NodeManager.o bin/GameSokoban.o bin/PriQueue.o bin/ScopeTimer.o
	$(CC) -std=c++11 -O3 -o sokoban bin/*.o -lm -lurlmon -Wall

clean:
	del /Q /S bin\*.o
	del /Q include\pcheaders.h.gch
	del /Q sokoban.exe
else
all: sokoban

sokoban: bin/main.o bin/Sokoban.o bin/BoxPositionSet.o bin/NodeManager.o bin/GameSokoban.o
	$(CC) -std=c++11 -O3 -o sokoban bin/*.o -lm -lcurl -Wall

clean:	
	rm -f sokoban
	rm -f bin/*.o
endif

bin/main.o: include/pcheaders.h include/Sokoban.h include/GameSokoban.h include/GameSokoban.h src/main.cpp
	$(CC) -std=c++11 -O3 -o bin/main.o -c src/main.cpp -Wall

bin/Sokoban.o: include/pcheaders.h include/StateSet.h include/Sokoban.h include/PriQueue.h include/BoxPositionSet.h include/NodeManager.h src/Sokoban.cpp
	$(CC) -std=c++11 -O3 -o bin/Sokoban.o -c src/Sokoban.cpp -Wall

bin/NodeManager.o: include/pcheaders.h include/NodeManager.h include/BoxPositionSet.h include/Sokoban.h src/NodeManager.cpp
	$(CC) -std=c++11 -O3 -o bin/NodeManager.o -c src/NodeManager.cpp -Wall

bin/StateSet.o: include/pcheaders.h include/StateSet.h include/BoxPositionSet.h include/Sokoban.h include/ScopeTimer.h src/StateSet.cpp
	$(CC) -std=c++11 -O3 -o bin/StateSet.o -c src/StateSet.cpp -Wall

bin/PriQueue.o: include/pcheaders.h include/BoxPositionSet.h include/PriQueue.h include/Sokoban.h src/PriQueue.cpp
	$(CC) -std=c++11 -O3 -o bin/PriQueue.o -c src/PriQueue.cpp -Wall

bin/BoxPositionSet.o: include/pcheaders.h include/BoxPositionSet.h src/BoxPositionSet.cpp
	$(CC) -std=c++11 -O3 -o bin/BoxPositionSet.o -c src/BoxPositionSet.cpp -Wall

bin/GameSokoban.o: include/pcheaders.h include/BoxPositionSet.h include/Sokoban.h include/GameSokoban.h src/GameSokoban.cpp
	$(CC) -std=c++11 -O3 -o bin/GameSokoban.o -c src/GameSokoban.cpp -Wall

bin/ScopeTimer.o: include/pcheaders.h include/ScopeTimer.h src/ScopeTimer.cpp
	$(CC) -std=c++11 -O3 -o bin/ScopeTimer.o -c src/ScopeTimer.cpp -Wall
