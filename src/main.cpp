#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/GameSokoban.h"

#include <thread>
#include <mutex>

#define TH_BUFFER_SIZE 10
std::mutex bufferMutex;

bool th_quitThread = false;
int th_bufferStart = 0, th_bufferEnd = 0;
int th_curBufferSize = 0;

std::vector<ACTION> th_solBuffer[TH_BUFFER_SIZE];
int th_levelNs[TH_BUFFER_SIZE];

void sendInputsThread(GameSokoban* gsb, bool saveOnly){
  while(!th_quitThread || th_curBufferSize > 0){
    while(th_curBufferSize == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    if(saveOnly){
      FILE *fp = fopen("SaveSolution.txt", "a");
      fprintf(fp, "%d\n", th_levelNs[th_bufferStart]);
      for(int i = 0; i < (int) th_solBuffer[th_bufferStart].size(); i++){
        switch(th_solBuffer[th_bufferStart][i]){
          case UP:
            fprintf(fp, "U");
            break;
          case DOWN:
            fprintf(fp, "D");
            break;
          case LEFT:
            fprintf(fp, "L");
            break;
          case RIGHT:
            fprintf(fp, "R");
            break;
        }
      }
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      gsb->sendSolution(th_levelNs[th_bufferStart], th_solBuffer[th_bufferStart]);
    }

    bufferMutex.lock();
    th_bufferStart = (th_bufferStart + 1) % TH_BUFFER_SIZE;
    th_curBufferSize--;
    bufferMutex.unlock();
  }
}

int intFromString(char *str){
  int result = 0;
  while(*str != '\0'){
    result = result * 10 + (*str - '0');
    str++;
  }
  return result;
}

void parseArgs(int argc, char* argv[], bool *showHelp, bool *verbose, bool *lowMemory, int *ramLimit, bool *nonIT, bool *nonITSave, char levelFile[],
               int *nonITStart, int *nonITEnd)
{
  int i = 1;

  while(i < argc){
    if(!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
      *showHelp = true;
    else if(!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose"))
      *verbose = true;
    else if(!strcmp(argv[i], "-lm") || !strcmp(argv[i], "--lowMemory"))
      *lowMemory = true;
    else if(!strcmp(argv[i], "--nonIT"))
      *nonIT = true;
     else if(!strcmp(argv[i], "--nonITSave"))
      *nonITSave = true;
    else if(!strcmp(argv[i], "--nonITStart")){
      i++;
      if(i < argc){
        *nonITStart = intFromString(argv[i]);
      }
    }
    else if(!strcmp(argv[i], "--nonITLast")){
      i++;
      if(i < argc){
        *nonITEnd = intFromString(argv[i]);
      }
    }
    else if(!strcmp(argv[i], "-f") || !strcmp(argv[i], "--file")){
      i++;
      if(i < argc){
        strcpy(levelFile, argv[i]);
      }
    }
    else if(!strcmp(argv[i], "-rl") || !strcmp(argv[i], "--ramLimit")){
      i++;
      if(i < argc){
        *ramLimit = intFromString(argv[i]);
        if(*ramLimit > 98 || *ramLimit < 1)
          *ramLimit = 95;
      }
    }
    else{
      printf("Warning: Unrecognized argument \"%s\"\n", argv[i]);
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
  printf("(-rl | --ramLimit) [N]    : Percentage of system RAM limt (1 - 98)\n");
  printf("(--nonIT)                 : Non interactive mode (game-sokoban.com)\n");
  printf("(--nonITSave)             : Non interactive mode (Save to file)\n");
  printf("(--nonITStart [N])        : First level of Non interactive mode\n");
  printf("(--nonITLast [N])         : Last level of Non interactive mode\n");
  printf("(-f | --file) [File]      : Level file path\n");
}

SOKOBAN_SOLUTION trySolve(bool verbose, bool lowMemory, int ramLimit, Sokoban *sokoban){
  SOKOBAN_SOLUTION sl = sokoban->solve(verbose, lowMemory, ramLimit);

  printf("\nInitial Heuristic: %d\n", sl.initialHeuristic);

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

void nonITAuto(const bool verbose, const bool lowMemory, const bool nonITSave, const int ramLimit, int curLev, const int lastLevel){
  int rb = 0;
  FILE *fp;

  if(curLev == -1){
    fp = fopen("GameSokobanData/data.txt", "r");
    if(fp){
      rb += fscanf(fp, "%d\n", &curLev);
      fclose(fp);
    }
    else{
      fprintf(stderr, "Could not open file: GameSokobanData/data.txt\n");
      exit(1);
    }
  }

  GameSokoban gsb = GameSokoban();
  std::thread sendThread(sendInputsThread, &gsb, nonITSave);

  while(curLev <= lastLevel && gsb.getLevel(curLev, "GameSokobanData/lev.txt")){
    printf("AUTO: Solving level %d...\n", curLev);
    FILE *fps = fopen("GameSokobanData/lev.txt", "r");
    Sokoban sokoban(fps);
    fclose(fps);

    SOKOBAN_SOLUTION solution;

    solution = trySolve(verbose, lowMemory, ramLimit, &sokoban);

    if(solution.cost > 0){
      while(th_curBufferSize == TH_BUFFER_SIZE)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

      bufferMutex.lock();
      th_levelNs[th_bufferEnd] = curLev;
      th_solBuffer[th_bufferEnd] = solution.path;
      th_bufferEnd = (th_bufferEnd + 1) % TH_BUFFER_SIZE;
      th_curBufferSize++;
      bufferMutex.unlock();
    }

    curLev++;
  }

  if(curLev > lastLevel){
    th_quitThread = true;
    sendThread.join();
  }
  else{
    printf("AUTO: Could not get level: %d\n", curLev);
  }
}

int main(int argc, char* argv[]){
  bool verbose = false;
  bool showHelp = false;
  bool lowMemory = false;
  bool nonIT = false;
  char levelFile[512] = "NONE_LEVEL";
  int ramLimit = 95;
  bool nonITSave = false;

  int nonITStart = -1, nonITLast = 9999999;

  parseArgs(argc, argv, &showHelp, &verbose, &lowMemory, &ramLimit, &nonIT, &nonITSave, levelFile, &nonITStart, &nonITLast);

  if(showHelp){
    showHelpMessage();
    return 0;
  }

  if(!strcmp("NONE_LEVEL", levelFile) && !nonIT){
    printf("No level file provided\n");
    return 1;
  }

  if(nonIT){
    nonITAuto(verbose, lowMemory, nonITSave, ramLimit, nonITStart, nonITLast);
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

    solution = trySolve(verbose, lowMemory, ramLimit, &sokoban);
  }

  return 0;
}
