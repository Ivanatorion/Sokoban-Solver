#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/GameSokoban.h"

int intFromString(char *str){
  int result = 0;
  while(*str != '\0'){
    result = result * 10 + (*str - '0');
    str++;
  }
  return result;
}

void parseArgs(int argc, char* argv[], bool *showHelp, bool *verbose, ALGO *algo, bool *lowMemory, int *ramLimit, bool *greedyOOM, bool *nonIT, char levelFile[]){
  int i = 0;

  while(i < argc){
    if(!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
      *showHelp = true;
    if(!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose"))
      *verbose = true;
    if(!strcmp(argv[i], "-lm") || !strcmp(argv[i], "--lowMemory"))
      *lowMemory = true;
    if(!strcmp(argv[i], "--greedyOOM"))
      *greedyOOM = true;
    if(!strcmp(argv[i], "--nonIT"))
      *nonIT = true;
    if(!strcmp(argv[i], "-f") || !strcmp(argv[i], "--file")){
      i++;
      if(i < argc){
        strcpy(levelFile, argv[i]);
      }
    }
    if(!strcmp(argv[i], "-a") || !strcmp(argv[i], "--algorithm")){
      i++;
      if(i < argc){
        if(!strcmp(argv[i], "astar"))
          *algo = ASTAR;
        if(!strcmp(argv[i], "idastar"))
          *algo = IDASTAR;
        if(!strcmp(argv[i], "greedy"))
          *algo = GREEDY;
        if(!strcmp(argv[i], "peastar"))
          *algo = PEASTAR;
      }
    }
    if(!strcmp(argv[i], "-rl") || !strcmp(argv[i], "--ramLimit")){
      i++;
      if(i < argc){
        *ramLimit = intFromString(argv[i]);
        if(*ramLimit > 98 || *ramLimit < 1)
          *ramLimit = 95;
      }
    }
    i++;
  }
}

void showHelpMessage(){
  printf("Sokoban Solver\n\nUsage: ./sokoban [Args] < level.txt\n");
  printf("\nArgs:\n");
  printf("(-h  | --help)            : Display help message\n");
  printf("(-v  | --verbose)         : Prints search status during execution\n");
  printf("(-lm | --lowMemory)       : Use less memory during search (at expense of search speed)\n");
  printf("(-a  | --algorithm) [Alg] : Specfies the algorithm to use in the search\n");
  printf("(-rl | --ramLimit) [N]    : Percentage of system RAM limt (1 - 98)\n");
  printf("(--greedyOOM)             : Retry with GBFS if out of memory\n");
  printf("(--nonIT)                 : Non interactive mode (game-sokoban.com)\n");
  printf("(-f | --file) [File]      : Level file path\n");
  printf("\nAlg: astar | idastar | greedy | peastar\n");
}

SOKOBAN_SOLUTION trySolve(bool verbose, bool lowMemory, ALGO algo, int ramLimit, Sokoban *sokoban){
  SOKOBAN_SOLUTION sl = sokoban->solve(verbose, lowMemory, algo, 0, ramLimit);

  printf("\nInitial Heuristic: %d\n", sl.heuristicaInicial);

  if(sl.cost >= 0){
    printf("Cost: %d\n", sl.cost);
    for(int i = 0; i < (int) sl.path.size(); i++){
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
  printf("Added %lld States to Open\nExpanded: %lld States\nTime(ms): %lld\n", sl.addedToOpen, sl.expanded, sl.timeMilis);

  return sl;
}

void nonITAuto(const bool verbose, const bool lowMemory, const ALGO algo, const int ramLimit, const bool greedyOOM){
  int curLev = 35594;
  int rb = 0;

  FILE *fp = fopen("GameSokobanData/data.txt", "r");
  if(fp){
    rb += fscanf(fp, "%d\n", &curLev);
    fclose(fp);
  }

  GameSokoban gsb = GameSokoban();
  while(gsb.getLevel(curLev, "GameSokobanData/lev.txt")){
    printf("AUTO: Solving level %d...\n", curLev);
    FILE *fps = fopen("GameSokobanData/lev.txt", "r");
    Sokoban sokoban(fps);
    fclose(fps);

    SOKOBAN_SOLUTION solution;

    solution = trySolve(verbose, lowMemory, algo, ramLimit, &sokoban);

    if(solution.cost == NO_RAM && greedyOOM && algo != GREEDY){
      printf("\nRetrying with GBFS...\n");
      solution = trySolve(verbose, lowMemory, GREEDY, ramLimit, &sokoban);
    }

    //gsb.sendSolution(curLev, solution.path);

    char buffer[1024];
    sprintf(buffer, "GameSokobanData/Solutions.txt");
    fps = fopen(buffer, "a");
    fprintf(fps, "Level: %d\n", curLev);
    if(solution.cost == NO_RAM){
      fprintf(fps, "Result: No RAM\n");
    }
    else if(solution.cost == NO_SOLUTION){
      fprintf(fps, "Result: No solution\n");
    }
    else{
      fprintf(fps, "Result: Solved (");
      switch(algo){
        case PEASTAR:
          fprintf(fps, "PEA*");
          break;
        case ASTAR:
          fprintf(fps, "A*");
          break;
        case GREEDY:
          fprintf(fps, "GBFS");
          break;
        case IDASTAR:
          fprintf(fps, "IDA*");
          break;
      }
      fprintf(fps, ")\nExpanded states: %lld\nTime(ms): %lld\nCost: %d\n", solution.expanded, solution.timeMilis, solution.cost);
      for(int i = 0; i < (int) solution.path.size(); i++){
        switch(solution.path[i]){
          case UP:
            fprintf(fps, "U");
            break;
          case DOWN:
            fprintf(fps, "D");
            break;
          case LEFT:
            fprintf(fps, "L");
            break;
          case RIGHT:
            fprintf(fps, "R");
            break;
        }
      }
      fprintf(fps, "\n\n");
    }
    fclose(fps);
    curLev++;
    fp = fopen("GameSokobanData/data.txt", "w");
    fprintf(fp, "%d\n", curLev);
    fclose(fp);
  }

  printf("AUTO: Could not get level: %d\n", curLev);
}

int main(int argc, char* argv[]){
  bool verbose = false;
  bool showHelp = false;
  bool greedyOOM = false;
  bool lowMemory = false;
  bool nonIT = false;
  char levelFile[512] = "NONE_LEVEL";
  int ramLimit = 95;

  ALGO algo = ASTAR;
  parseArgs(argc, argv, &showHelp, &verbose, &algo, &lowMemory, &ramLimit, &greedyOOM, &nonIT, levelFile);

  if(showHelp){
    showHelpMessage();
    return 0;
  }

  if(!strcmp("NONE_LEVEL", levelFile) && !nonIT){
    printf("No level file provided\n");
    return 1;
  }

  if(nonIT){
    nonITAuto(verbose, lowMemory, algo, ramLimit, greedyOOM);
  }
  else{
    FILE *fps = fopen(levelFile, "r");
    if(!fps){
      printf("Could not open file: %s\n", levelFile);
      return 1;
    }
    Sokoban sokoban(fps);
    fclose(fps);

    SOKOBAN_SOLUTION solution;

    solution = trySolve(verbose, lowMemory, algo, ramLimit, &sokoban);

    if(solution.cost == NO_RAM && greedyOOM && algo != GREEDY){
      printf("\nRetrying with GBFS...\n");
      solution = trySolve(verbose, lowMemory, GREEDY, ramLimit, &sokoban);
    }
  }

  return 0;
}
