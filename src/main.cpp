#include <cstdio>
#include <cstring>

#include "../include/Sokoban.h"

void parseArgs(int argc, char* argv[], bool *showHelp, bool *verbose, ALGO *algo, bool *lowMemory){
  int i = 0;

  while(i < argc){
    if(!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
      *showHelp = true;
    if(!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose"))
      *verbose = true;
    if(!strcmp(argv[i], "-lm") || !strcmp(argv[i], "--lowMemory"))
      *lowMemory = true;
    if(!strcmp(argv[i], "-a") || !strcmp(argv[i], "--algorithm")){
      i++;
      if(i < argc){
        if(!strcmp(argv[i], "astar"))
          *algo = ASTAR;
        if(!strcmp(argv[i], "idastar"))
          *algo = IDASTAR;
        if(!strcmp(argv[i], "greedy"))
          *algo = GREEDY;
      }
    }
    i++;
  }
}

void showHelpMessage(){
  printf("Sokoban Solver\n\nUsage: ./sokoban [Args] < level.txt\n");
  printf("\nArgs:\n");
  printf("(-h  | --help )            : Display help message\n");
  printf("(-v  | --verbose )         : Prints search status during execution\n");
  printf("(-lm | --lowMemory )       : Use less memory during search (at expense of search speed)\n");
  printf("(-a  | --algorithm ) [Alg] : Specfies the algorithm to use in the search\n");
  printf("\nAlg: astar | idastar | greedy\n");
}

int main(int argc, char* argv[]){
  bool verbose = false;
  bool showHelp = false;
  bool greedy = false;
  bool lowMemory = false;
  ALGO algo = ASTAR;
  parseArgs(argc, argv, &showHelp, &verbose, &algo, &lowMemory);

  if(showHelp){
    showHelpMessage();
    return 0;
  }

  Sokoban sokoban(stdin);

  SOKOBAN_SOLUTION sl = sokoban.solve(verbose, lowMemory, greedy, algo);

  printf("Initial Heuristic: %d\n", sl.heuristicaInicial);

  if(sl.cost >= 0){
    printf("Cost: %d\n", sl.cost);
    for(int i = 0; i < sl.path.size(); i++){
      switch(sl.path[i]){
        case UP:
          printf("U");
          break;
        case DOWN:
          printf("D");
          break;
        case LEFT:
          printf("L");
          break;
        case RIGHT:
          printf("R");
          break;
      }
    }
    printf("\n");
  }
  else{
    switch (sl.cost) {
      case NO_SOLUTION:
        printf("Problem is unsolvable!\n");
        break;
      case NO_RAM:
        printf("Ram limit achieved!\n");
        break;
    }
  }
  printf("Expanded: %lld States\nTime(ms): %lld\n", sl.expanded, sl.timeMilis);

  return 0;
}
