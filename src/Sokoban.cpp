#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/NodeManager.h"

#ifdef _WIN32
#include <sys/types.h>
#include <windows.h>
#endif

#ifdef __linux__
#include<unistd.h>

typedef uint32_t DWORD;
typedef uint64_t DWORDLONG;

typedef struct _MEMORYSTATUSEX {
  DWORD     dwLength;
  DWORD     dwMemoryLoad;
  /*
  DWORDLONG ullTotalPhys;
  DWORDLONG ullAvailPhys;
  DWORDLONG ullTotalPageFile;
  DWORDLONG ullAvailPageFile;
  DWORDLONG ullTotalVirtual;
  DWORDLONG ullAvailVirtual;
  DWORDLONG ullAvailExtendedVirtual;
  */
} MEMORYSTATUSEX;

int _lin_int_from_string(char *str){
  int result = 0;
  while(*str != '\0'){
    result = result * 10 + (*str - '0');
    str++;
  }
  return result;
}

void GlobalMemoryStatusEx(MEMORYSTATUSEX* msx){
  static DWORD result = 0;
  static time_t lastCheck = 0;

  time_t cTime = time(0);
  if(cTime - lastCheck < 5){
    msx->dwMemoryLoad = result;
    return;
  }
  lastCheck = cTime;

  char c;
  char bufAv[256];
  char bufTot[256];
  int auxI = 0;
  FILE* fp = NULL;
  do{
    fp = popen("head -n3 /proc/meminfo", "r");
    if(fp){
      c = fgetc(fp);
      if(c != 'M'){
        pclose(fp);
        fp = NULL;
      }
    }
  }while(fp == NULL);

  while(c < '0' || c > '9')
      c = fgetc(fp);
  while(!(c < '0' || c > '9')){
      bufTot[auxI] = c;
      auxI++;
      c = fgetc(fp);
  }
  bufTot[auxI] = '\0';
  auxI = 0;
  while(c != '\n')
      c = fgetc(fp);
  c = fgetc(fp);
  while(c != '\n')
      c = fgetc(fp);
  while(c < '0' || c > '9')
      c = fgetc(fp);
  while(!(c < '0' || c > '9')){
      bufAv[auxI] = c;
      auxI++;
      c = fgetc(fp);
  }
  bufAv[auxI] = '\0';
  pclose(fp);

  const int mTotal = _lin_int_from_string(bufTot);
  const int mAvail = _lin_int_from_string(bufAv);
  const int mUsed = mTotal - mAvail;

  msx->dwMemoryLoad = (mUsed * 100) / mTotal;
  result = msx->dwMemoryLoad;
}
#endif

int BoxPositionSet::STATE_BOX_QUANT = 0;

Sokoban::Sokoban(FILE* inputMap){
  this->ramLimit = RAM_LIMIT;

  std::vector<std::vector<char>> tempGameMap;
  std::vector<char> line;
  char c;
  int biggestLine = 0;

  while(!feof(inputMap)){
    c = fgetc(inputMap);
    line.clear();
    while(c != '\n' && !feof(inputMap)){
      line.push_back(c);
      c = fgetc(inputMap);
    }
    if(line.size() > 0 && line[0] != '/'){
      for(char &c : line)
        if(c != EMPTY && c != WALL && c != GOAL && c != BOX && c != PLAYER && c != GOAL_AND_BOX && c != PLAYER_AND_GOAL)
          c = WALL;

      tempGameMap.push_back(line);

      if((int) line.size() > biggestLine)
        biggestLine = (int) line.size();
    }
  }

  for(std::vector<char> &l : tempGameMap)
    l.resize(biggestLine, WALL);

  this->mapLenX = biggestLine;
  this->mapLenY = tempGameMap.size();

  this->gameMap = new char[this->mapLenX * this->mapLenY];
  this->isDeadEnd = new bool[this->mapLenX * this->mapLenY];
  this->isGoal = new bool[this->mapLenX * this->mapLenY];
  this->minGoalDistance = new int[this->mapLenX * this->mapLenY];

  this->mapBoxQuant = 0;
  for(int y = 0; y < this->mapLenY; y++){
    for(int x = 0; x < this->mapLenX; x++){
      this->gameMap[x + this->mapLenX * y] = tempGameMap[y][x];
      this->isGoal[x + this->mapLenX * y] = (this->gameMap[x + this->mapLenX * y] == GOAL || this->gameMap[x + this->mapLenX * y] == GOAL_AND_BOX || this->gameMap[x + this->mapLenX * y] == PLAYER_AND_GOAL );
      if(this->isGoal[x + this->mapLenX * y])
        this->mapBoxQuant++;
    }
  }

  BoxPositionSet::STATE_BOX_QUANT = this->mapBoxQuant;

  this->mmMatrix = new int[this->mapBoxQuant * this->mapBoxQuant];
  this->staticmmMatrix = new int[this->mapBoxQuant * this->mapBoxQuant];
  this->goalAssigned = new int[this->mapBoxQuant];
  this->boxAssigned = new int[this->mapBoxQuant];
  this->couldntAssignBox = new bool[this->mapBoxQuant];
  this->couldntAssignGoal = new bool[this->mapBoxQuant];
  this->numberZeroesRow = new int[this->mapBoxQuant];
  this->numberZeroesCol = new int[this->mapBoxQuant];
  this->tickedRows = new bool[this->mapBoxQuant];
  this->tickedCols = new bool[this->mapBoxQuant];

  this->treatWall = new bool[this->mapLenX * this->mapLenY];

  this->distBoxesPP = new int[this->mapLenX * this->mapLenY];
  this->distBoxesNPP = new int[this->mapLenX * this->mapLenY];

  std::queue<std::pair<POSITION, int>> q;
  std::unordered_set<POSITION> closed;
  for(POSITION mapPos = 0; mapPos < this->mapLenX * this->mapLenY; mapPos++){
    if(this->gameMap[mapPos] == WALL){
      this->minGoalDistance[mapPos] = SHRT_MAX;
      this->isDeadEnd[mapPos] = false;
      continue;
    }

    while(!q.empty()) q.pop();
    closed.clear();

    bool foundGoal = false;

    std::pair<POSITION, int> auxP;
    auxP.first = mapPos;
    auxP.second = 0;
    int distMin = 0;

    q.push(auxP);
    while(!q.empty() && !foundGoal){
      std::pair<POSITION, int> p = q.front();
      q.pop();

      if(closed.find(p.first) == closed.end()){
        closed.insert(p.first);

        if(!this->isGoal[p.first]){
          POSITION neighboor = p.first;
          auxP.second = p.second + 1;

          neighboor = neighboor - this->mapLenX;
          if(neighboor >= 0 && gameMap[neighboor] != WALL && (neighboor + 2*this->mapLenX) < this->mapLenX*this->mapLenY && this->gameMap[neighboor + 2*this->mapLenX] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
          neighboor = neighboor + 2*this->mapLenX;
          if(neighboor < this->mapLenX*this->mapLenY && gameMap[neighboor] != WALL && (neighboor - 2*this->mapLenX) >=0 && this->gameMap[neighboor - 2*this->mapLenX] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
          neighboor = neighboor - this->mapLenX;

          if(neighboor % this->mapLenX != 0){
            neighboor--;
            if(gameMap[neighboor] != WALL && (neighboor + 2) % this->mapLenX != 0 && this->gameMap[neighboor + 2] != WALL){
              auxP.first = neighboor;
              q.push(auxP);
            }
            neighboor++;
          }
          if((neighboor+1) % this->mapLenX != 0){
            neighboor++;
            if(gameMap[neighboor] != WALL && (neighboor - 1) % this->mapLenX != 0 &&  this->gameMap[neighboor - 2] != WALL){
              auxP.first = neighboor;
              q.push(auxP);
            }
          }
        }
        else{
          distMin = p.second;
          foundGoal = true;
        }
      }
    }
    minGoalDistance[mapPos] = (foundGoal) ? distMin : SHRT_MAX;
    isDeadEnd[mapPos] = (minGoalDistance[mapPos] == SHRT_MAX);
  }
  while(!q.empty()) q.pop();

  //Calculate Tile Distances
  this->tileDistances = new int[this->mapLenX * this->mapLenY * this->mapLenX * this->mapLenY];
  calculateTileDistances();
  this->boxTileDistances = new int[this->mapLenX * this->mapLenY * this->mapLenX * this->mapLenY];
  //calculateBoxTileDistances();

  //Updates minGoalDistance
  goalPositions.clear();
  for(POSITION p = 0; p < this->mapLenX * this->mapLenY; p++){
    if(this->isGoal[p])
      goalPositions.push_back(p);
  }

  for(POSITION p = 0; p < this->mapLenX * this->mapLenY; p++){
    this->minGoalDistance[p] = SHRT_MAX;
    for(POSITION gp : goalPositions)
      if(getTileDistance(p, gp) < this->minGoalDistance[p])
        this->minGoalDistance[p] = getTileDistance(p, gp);
  }

  /*
  printf("Map:\n");
  for(int y = 0; y < this->mapLenY; y++){
    for(int x = 0; x < this->mapLenX; x++){
      printf("%c", this->gameMap[x + this->mapLenX * y]);
    }
    printf("\n");
  }
  printf("\nGoals: ");
  for(int y = 0; y < this->mapLenY; y++){
    for(int x = 0; x < this->mapLenX; x++){
      if(this->isGoal[x + this->mapLenX * y])
        printf("(%d %d) ", x, y);
    }
  }
  printf("\nDead Ends: ");
  for(int y = 0; y < this->mapLenY; y++){
    for(int x = 0; x < this->mapLenX; x++){
      if(this->isDeadEnd[x + this->mapLenX * y])
        printf("(%d %d) ", x, y);
    }
  }
  printf("\n");
  printf("Min Goal Distances:\n");
  for(int y = 0; y < this->mapLenY; y++){
    for(int x = 0; x < this->mapLenX; x++){
      if(this->minGoalDistance[x + this->mapLenX*y] == SHRT_MAX)
        printf("XX ");
      else
        printf("%02d ", this->minGoalDistance[x + this->mapLenX*y]);
    }
    printf("\n\n");
  }
  exit(0);
  */

}

Sokoban::~Sokoban(){
  delete[] this->gameMap;
  delete[] this->isDeadEnd;
  delete[] this->isGoal;
  delete[] this->minGoalDistance;
  delete[] this->tileDistances;
  delete[] this->boxTileDistances;

  delete[] this->mmMatrix;
  delete[] this->staticmmMatrix;
  delete[] this->goalAssigned;
  delete[] this->boxAssigned;
  delete[] this->couldntAssignBox;
  delete[] this->couldntAssignGoal;
  delete[] this->numberZeroesRow;
  delete[] this->numberZeroesCol;
  delete[] this->tickedRows;
  delete[] this->tickedCols;

  delete[] this->treatWall;

  delete[] this->distBoxesPP;
  delete[] this->distBoxesNPP;
}

struct ComparePriorityAstar {
    bool operator()(SOKOBAN_NODE* const& n1, SOKOBAN_NODE* const& n2)
    {
        if(n1->pathCost + n1->heuristica + n1->additionalF == n2->pathCost + n2->heuristica + n2->additionalF)
          return n1->heuristica > n2->heuristica;
        else
          return n1->pathCost + n1->heuristica + n1->additionalF > n2->pathCost + n2->heuristica + n2->additionalF;
    }
};

struct ComparePriorityGreedy {
    bool operator()(SOKOBAN_NODE* const& n1, SOKOBAN_NODE* const& n2)
    {
        return n1->heuristica > n2->heuristica;
    }
};

int Sokoban::fHeuristica(SOKOBAN_STATE &state){
  int result = 0;

  /*
  //Calculate player distance to closest box
  int d = SHRT_MAX;
  int d2;
  POSITION border;
  for(POSITION bp : state.boxPositions){
    border = bp - this->mapLenX;
    if(border >= 0){
      d2 = getTileDistance(state.playerPosition, border);
      if(d2 < d)
        d = d2;
    }
    border = bp + this->mapLenX;
    if(border < this->mapLenX * this->mapLenY){
      d2 = getTileDistance(state.playerPosition, border);
      if(d2 < d)
        d = d2;
    }
    if(bp % this->mapLenX != 0){
      border = bp - 1;
      d2 = getTileDistance(state.playerPosition, border);
      if(d2 < d)
        d = d2;
    }
    if((bp+1) % this->mapLenX != 0){
      border = bp + 1;
      d2 = getTileDistance(state.playerPosition, border);
      if(d2 < d)
        d = d2;
    }
  }
  result = result + d;
  */

  for(POSITION bp : state.boxPositions)
    result = result + this->minGoalDistance[bp];
  return result;

  //Minimum Matching assignment
  int row = 0;
  int minimum;
  for(POSITION bp : state.boxPositions){
    for(int i = 0; i < this->mapBoxQuant; i++){
      this->mmMatrix[i + row] = getTileDistance(bp, goalPositions[i]);
      this->staticmmMatrix[i + row] = this->mmMatrix[i + row];
    }
    row = row + this->mapBoxQuant;
  }

  //Row minimum subtraction
  for(int i = 0; i < this->mapBoxQuant; i++){
    minimum = mmMatrix[i * this->mapBoxQuant];
    for(int j = 1; j < this->mapBoxQuant; j++){
      if(mmMatrix[j + i * this->mapBoxQuant] < minimum)
        minimum = mmMatrix[j + i * this->mapBoxQuant];
    }
    for(int j = 0; j < this->mapBoxQuant; j++)
      mmMatrix[j + i * this->mapBoxQuant] = mmMatrix[j + i * this->mapBoxQuant] - minimum;
  }

  //Columm minimum subtraction
  for(int j = 0; j < this->mapBoxQuant; j++){
    minimum = mmMatrix[j];
    for(int i = 1; i < this->mapBoxQuant; i++){
      if(mmMatrix[j + i * this->mapBoxQuant] < minimum)
        minimum = mmMatrix[j + i * this->mapBoxQuant];
    }
    for(int i = 0; i < this->mapBoxQuant; i++)
      mmMatrix[j + i * this->mapBoxQuant] = mmMatrix[j + i * this->mapBoxQuant] - minimum;
  }

  bool foundAssing = false;

  while(!foundAssing){
    for(int i = 0; i < this->mapBoxQuant; i++){
      numberZeroesRow[i] = 0;
      numberZeroesCol[i] = 0;
    }

    for(int i = 0; i < this->mapBoxQuant; i++){
      goalAssigned[i] = -1;
      boxAssigned[i] = -1;
      couldntAssignBox[i] = false;
      couldntAssignGoal[i] = false;
      for(int j = 0; j < this->mapBoxQuant; j++){
        if(mmMatrix[i * this->mapBoxQuant + j] == 0){
          numberZeroesRow[i]++;
          numberZeroesCol[j]++;
        }
      }
    }

    bool stopAssingment = false;
    while(!stopAssingment){
      bool rowAsg;
      int minimumRow = 0;
      while(minimumRow < this->mapBoxQuant && (goalAssigned[minimumRow] != -1 || couldntAssignBox[minimumRow])){
        minimumRow++;
      }
      for(int j = minimumRow + 1; j < this->mapBoxQuant; j++){
        if(goalAssigned[j] == -1 && !couldntAssignBox[j] && numberZeroesRow[j] < numberZeroesRow[minimumRow])
          minimumRow = j;
      }

      int minimumCol = 0;
      while(minimumCol < this->mapBoxQuant && (boxAssigned[minimumCol] != -1 || couldntAssignGoal[minimumCol])){
        minimumCol++;
      }
      for(int j = minimumCol + 1; j < this->mapBoxQuant; j++){
        if(boxAssigned[j] == -1 && !couldntAssignGoal[j] && numberZeroesCol[j] < numberZeroesCol[minimumCol])
          minimumCol = j;
      }

      if(minimumRow >= this->mapBoxQuant || (minimumCol < this->mapBoxQuant && numberZeroesCol[minimumCol] < numberZeroesRow[minimumRow])){
        rowAsg = false;
        minimum = minimumCol;
      }
      else{
        rowAsg = true;
        minimum = minimumRow;
      }

      if(rowAsg){
        int col = 0;
        int chosenCol = -1;
        while(col < this->mapBoxQuant){
          if(mmMatrix[minimum * this->mapBoxQuant + col] == 0){
            bool taken = false;
            for(int k = 0; k <  this->mapBoxQuant; k++)
              if(goalAssigned[k] == col)
                taken = true;
            if(!taken && (chosenCol == -1 || numberZeroesCol[col] < numberZeroesCol[chosenCol]))
              chosenCol = col;
          }
          col++;
        }
        if(chosenCol != -1){
          goalAssigned[minimum] = chosenCol;
          boxAssigned[chosenCol] = minimum;
          for(int k = 0; k < this->mapBoxQuant; k++)
            if(mmMatrix[chosenCol + k * this->mapBoxQuant] == 0)
              numberZeroesRow[k]--;
          for(int k = 0; k < this->mapBoxQuant; k++)
            if(mmMatrix[minimum * this->mapBoxQuant + k] == 0)
              numberZeroesCol[k]--;
        }

        if(goalAssigned[minimum] == -1)
          couldntAssignBox[minimum] = true;
      }
      else{
        int row = 0;
        int chosenRow = -1;
        while(row < this->mapBoxQuant){
          if(mmMatrix[row * this->mapBoxQuant + minimum] == 0){
            bool taken = false;
            for(int k = 0; k <  this->mapBoxQuant; k++)
              if(boxAssigned[k] == row)
                taken = true;
            if(!taken && (chosenRow == -1 || numberZeroesRow[row] < numberZeroesRow[chosenRow]))
              chosenRow = row;
          }
          row++;
        }
        if(chosenRow != -1){
          boxAssigned[minimum] = chosenRow;
          goalAssigned[chosenRow] = minimum;
          for(int k = 0; k < this->mapBoxQuant; k++)
            if(mmMatrix[k + chosenRow * this->mapBoxQuant] == 0)
              numberZeroesCol[k]--;
          for(int k = 0; k < this->mapBoxQuant; k++)
            if(mmMatrix[minimum + this->mapBoxQuant * k] == 0)
              numberZeroesRow[k]--;
        }

        if(boxAssigned[minimum] == -1)
          couldntAssignGoal[minimum] = true;
      }

      stopAssingment = true;
      for(int k = 0; k < this->mapBoxQuant; k++){
        if(!couldntAssignBox[k] && goalAssigned[k] == -1)
          stopAssingment = false;
      }
    }

    foundAssing = true;
    for(int i = 0; i < this->mapBoxQuant; i++)
      if(goalAssigned[i] == -1)
        foundAssing = false;

    if(!foundAssing){
      for(int i = 0; i < this->mapBoxQuant; i++){
        tickedRows[i] = (goalAssigned[i] == -1);
        tickedCols[i] = false;
      }

      bool keepTicking = true;

      while(keepTicking){
        keepTicking = false;
        for(int i = 0; i < this->mapBoxQuant; i++){
          if(tickedRows[i]){
            for(int j = 0; j < this->mapBoxQuant; j++)
              if(this->mmMatrix[i * this->mapBoxQuant + j] == 0)
                tickedCols[j] = true;
          }
        }

        for(int i = 0; i < this->mapBoxQuant; i++){
          if(tickedCols[i]){
            for(int j = 0; j < this->mapBoxQuant; j++){
              if(goalAssigned[j] == i && !tickedRows[j]){
                keepTicking = true;
                tickedRows[j] = true;
              }
            }
          }
        }
      }

      int theta = SHRT_MAX;
      for(int i = 0; i < this->mapBoxQuant; i++){
        if(tickedRows[i]){
          for(int j = 0; j < this->mapBoxQuant; j++){
            if(!tickedCols[j] && this->mmMatrix[i * this->mapBoxQuant + j] < theta)
              theta = this->mmMatrix[i * this->mapBoxQuant + j];
          }
        }
      }

      int nTickRows = 0;
      for(int i = 0; i < this->mapBoxQuant; i++)
        if(tickedRows[i])
          nTickRows++;
      if(nTickRows == this->mapBoxQuant){
        for(int i = 0; i < this->mapBoxQuant; i++){
          goalAssigned[i] = -1;
          boxAssigned[i] = -1;
        }
        forceAssing(0);
        foundAssing = true;
      }

      for(int i = 0; i < this->mapBoxQuant; i++){
        for(int j = 0; j < this->mapBoxQuant; j++){
          if(tickedRows[i] && !tickedCols[j])
            this->mmMatrix[i * this->mapBoxQuant + j] = this->mmMatrix[i * this->mapBoxQuant + j] - theta;
          if(!tickedRows[i] && tickedCols[j])
            this->mmMatrix[i * this->mapBoxQuant + j] = this->mmMatrix[i * this->mapBoxQuant + j] + theta;
        }
      }
    }
  }
  for(int i = 0; i < this->mapBoxQuant; i++)
    result = result + this->staticmmMatrix[i * this->mapBoxQuant + goalAssigned[i]];

  return result;
}

bool Sokoban::forceAssing(int col){
  if(col >= this->mapBoxQuant)
    return true;

  for(int i = 0; i < this->mapBoxQuant; i++){
    if(this->mmMatrix[i * this->mapBoxQuant + col] == 0){
      bool rowTaken = false;
      for(int j = col-1; j >= 0; j--)
        if(boxAssigned[j] == i)
          rowTaken = true;

      if(!rowTaken){
        boxAssigned[col] = i;
        goalAssigned[i] = col;
        if(forceAssing(col+1))
          return true;
        else{
          boxAssigned[col] = -1;
          goalAssigned[i] = -1;
        }
      }
    }
  }

  return false;
}

SOKOBAN_SOLUTION Sokoban::extractPath(SOKOBAN_NODE* n, std::vector<SOKOBAN_NODE*> &nodes, long long int expanded, long long int addedToOpen){
  SOKOBAN_SOLUTION sol;
  sol.cost = n->pathCost;
  sol.expanded = expanded;
  sol.addedToOpen = addedToOpen;

  while(n->parent != NULL){
    sol.path.push_back(n->action);
    n = n->parent;
  }

  for(int i = 0; i < (int) sol.path.size()/2; i++){
    ACTION aux = sol.path[i];
    sol.path[i] = sol.path[sol.path.size()-1-i];
    sol.path[sol.path.size()-1-i] = aux;
  }

  for(SOKOBAN_NODE* np : nodes){
    if(np->state != nullptr)
      delete np->state;

    delete np;
  }

  SOKOBAN_NODE* dummyNode = makeRootNode();
  sol.heuristicaInicial = dummyNode->heuristica;
  delete dummyNode->state;
  delete dummyNode;

  return sol;
}

SOKOBAN_SOLUTION Sokoban::solve(bool verbose, bool lowMemory, ALGO algo, int idastarFlimit, int ramLimit){
  auto tempoInicial = std::chrono::steady_clock::now();

  SOKOBAN_SOLUTION solution;

  this->lowMemory = lowMemory;

  if(ramLimit >= 1 && ramLimit <= 98){
    this->ramLimit = ramLimit;
  }

  switch (algo) {
    case ASTAR:
      solution = solveAstar(verbose);
      break;
    case PEASTAR:
      solution = solvePEAstar(verbose);
      break;
    case IDASTAR:
      solution = solveIdAstar(verbose, idastarFlimit);
      break;
    case GREEDY:
      solution = solveGBFS(verbose);
      break;
  }


  auto tempoFinal = std::chrono::steady_clock::now();
  long long int tempoTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(tempoFinal - tempoInicial).count();
  solution.timeMilis = tempoTotal;

  return solution;
}

std::pair<int, std::vector<ACTION>> Sokoban::recursiveSearchIdastar(SOKOBAN_NODE* node, int fLimit, int *expanded, bool *noneSol, std::unordered_set<SOKOBAN_STATE> &transpositionTable){
  std::pair<int, std::vector<ACTION>> solution;
  std::vector<ACTION> caminho;

  if(node->pathCost + node->heuristica > fLimit)
    return std::make_pair(node->pathCost + node->heuristica, caminho);

  if(isGoalState(*(node->state))){
    *noneSol = false;
    return std::make_pair(fLimit, caminho);
  }

  *expanded = *expanded + 1;
  int nextLimit = SHRT_MAX;
  for(std::pair<ACTION, SOKOBAN_STATE> suc : getSucc(*(node->state))){

    if(!movedBox(*(node->state), node->action) && suc.first == opposite(node->action))
      continue;

    if(transpositionTable.find(suc.second) != transpositionTable.end())
      continue;

    SOKOBAN_NODE* nl = makeNode(node, suc.first, suc.second);
    if(nl->heuristica != SHRT_MAX){
      transpositionTable.insert(*(nl->state));

      solution = recursiveSearchIdastar(nl, fLimit, expanded, noneSol, transpositionTable);

      transpositionTable.erase(*(nl->state));
      delete nl->state;
      delete nl;

      if(!(*noneSol)){
        caminho = solution.second;
        caminho.push_back(suc.first);
        return std::make_pair(solution.first, caminho);
      }
      if(solution.first < nextLimit)
        nextLimit = solution.first;
    }
    else{
      delete nl->state;
      delete nl;
    }
  }
  return std::make_pair(nextLimit, caminho);
}

SOKOBAN_SOLUTION Sokoban::solveIdAstar(bool verbose, int idastarFlimit){
  std::pair<int, std::vector<ACTION>> solution;

  SOKOBAN_NODE* root = makeRootNode();

  int fLimit = idastarFlimit <= 0 ? root->heuristica : idastarFlimit;
  int heuristicaInicial = fLimit;

  int expanded = 0;
  bool noneSol = true;

  std::unordered_set<SOKOBAN_STATE> transpositionTable;

  if(verbose)
    printf("New F-Limit: %d\n", fLimit);

  while(fLimit != SHRT_MAX){
    auto iterInit = std::chrono::steady_clock::now();

    solution = recursiveSearchIdastar(root, fLimit, &expanded, &noneSol, transpositionTable);
    if(!noneSol){
      delete root->state;
      delete root;
      SOKOBAN_SOLUTION sol;
      sol.heuristicaInicial = heuristicaInicial;
      sol.cost = solution.first;
      sol.expanded = expanded;
      sol.path = solution.second;
      for(int i = 0; i < (int) sol.path.size()/2; i++){
        ACTION auxA = sol.path[i];
        sol.path[i] = sol.path[sol.path.size()-1-i];
        sol.path[sol.path.size()-1-i] = auxA;
      }
      return sol;
    }
    else
      fLimit = solution.first;

    auto iterEnd = std::chrono::steady_clock::now();
    int iterTotal = (int) std::chrono::duration_cast<std::chrono::seconds>(iterEnd - iterInit).count();

    if(verbose)
      printf("New F-Limit: %d (%02d:%02d:%02d)\n", fLimit, iterTotal / 3600, (iterTotal % 3600) / 60, iterTotal % 60);

  }

  delete root->state;
  delete root;

  SOKOBAN_SOLUTION nosol;
  nosol.path.clear();
  nosol.cost = -1;
  nosol.expanded = expanded;
  nosol.heuristicaInicial = heuristicaInicial;
  nosol.addedToOpen = 0;
  nosol.timeMilis = 0;
  return nosol;
}

SOKOBAN_SOLUTION Sokoban::solveAstar(bool verbose){
  auto expandTestInicial = std::chrono::steady_clock::now();

  long long int expanded = 0;
  long long int addedToOpen = 0;
  std::priority_queue<SOKOBAN_NODE*, std::vector<SOKOBAN_NODE*>, ComparePriorityAstar> open;

  std::unordered_set<SOKOBAN_STATE> closed;

  closed.reserve(6000000);
  closed.max_load_factor(0.9);

  std::vector<SOKOBAN_NODE*> nodes;

  SOKOBAN_NODE* initialNode = makeRootNode();
  nodes.push_back(initialNode);

  int heuristicaInicial = initialNode->heuristica;

  //Verbose stuff
  int lowestH = SHRT_MAX;
  int highestF = 0;
  int stateExpandDelay = 1000;

  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);

  if(initialNode->heuristica < SHRT_MAX){
    open.push(initialNode);
    addedToOpen++;
  }

  while(!open.empty()){
    SOKOBAN_NODE* n = open.top();
    open.pop();

    if(verbose && n->heuristica < lowestH){
      lowestH = n->heuristica;
      printf("Lowest Heuristic: %d\n", lowestH);
    }
    if(verbose && n->pathCost + n->heuristica > highestF){
      highestF = n->pathCost + n->heuristica;
      printf("Highest F-Value: %d\n", highestF);
    }

    if(closed.find(*(n->state)) == closed.end()){
      closed.insert(*(n->state));

      if(isGoalState(*(n->state)))
        return extractPath(n, nodes, expanded, addedToOpen);

      expanded++;

      GlobalMemoryStatusEx (&statex);

      if(statex.dwMemoryLoad >= static_cast<unsigned int>(this->ramLimit)){
        if(verbose){
          printf("Out of memory! Terminating...\n");
        }
        
        for(SOKOBAN_NODE* n : nodes){
          delete n->state;
          delete n;
        }

        SOKOBAN_SOLUTION nosol;
        nosol.path.clear();
        nosol.cost = NO_RAM;
        nosol.expanded = expanded;
        nosol.heuristicaInicial = heuristicaInicial;
        nosol.addedToOpen = addedToOpen;
        nosol.timeMilis = 0;

        return nosol;
      }

      if(verbose && expanded % stateExpandDelay == 0){
        auto expandTestFinal = std::chrono::steady_clock::now();
        long long int expandTestTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(expandTestFinal - expandTestInicial).count();
        printf("Expanded: %lld States (Mem: %ld / %d)\n", expanded, statex.dwMemoryLoad, this->ramLimit);
        if(expandTestTotal < 3000)
          stateExpandDelay = stateExpandDelay*10;
        if(expandTestTotal > 30000)
          stateExpandDelay = stateExpandDelay/2;
        expandTestInicial = expandTestFinal;
      }

      for(std::pair<ACTION, SOKOBAN_STATE> s : getSucc(*(n->state))){

        if(closed.find(s.second) == closed.end()){
          if(this->lowMemory && !movedBox(s.second, s.first) && !checkMoveCloser(*(n->state), s.first)){
            closed.insert(s.second);
          }
          else{
            SOKOBAN_NODE* nl = makeNode(n, s.first, s.second);
            if(nl->heuristica < SHRT_MAX){
              nodes.push_back(nl);
              open.push(nl);
              addedToOpen++;
            }
            else{
              delete nl->state;
              delete nl;
            }
          }
        }
      }

      delete n->state;
      n->state = nullptr;
    }

  }

  for(SOKOBAN_NODE* n : nodes){
    delete n->state;
    delete n;
  }

  SOKOBAN_SOLUTION nosol;
  nosol.path.clear();
  nosol.cost = NO_SOLUTION;
  nosol.expanded = expanded;
  nosol.heuristicaInicial = heuristicaInicial;
  nosol.addedToOpen = addedToOpen;
  nosol.timeMilis = 0;

  if(!open.empty())
    nosol.cost = NO_RAM;

  return nosol;
}

/*
void deleteNodes(std::vector<SOKOBAN_NODE*>& nodes, std::unordered_set<SOKOBAN_NODE*>& toDelete){
  const int nSize = (int) nodes.size();
  int deletedCount = 0;
  for(int i = 0; i < nSize; i++){
    if(toDelete.find(nodes[i]) != toDelete.end()){
      deletedCount++;
      if(nodes[i]->parent != nullptr)
        nodes[i]->parent->nSuccs = nodes[i]->parent->nSuccs - 1;
      delete nodes[i]->state;
      delete nodes[i];
    }
    else{
      nodes[i - deletedCount] = nodes[i];
    }
  }
  nodes.erase(nodes.begin() + nodes.size() - deletedCount, nodes.begin() + nodes.size());
}
*/

SOKOBAN_SOLUTION Sokoban::solvePEAstar(bool verbose){
  auto expandTestInicial = std::chrono::steady_clock::now();

  long long int expanded = 0;
  long long int addedToOpen = 0;
  long long int prevOpenSize = 1;
  std::priority_queue<SOKOBAN_NODE*, std::vector<SOKOBAN_NODE*>, ComparePriorityAstar> open;

  std::unordered_set<SOKOBAN_STATE> closed;

  const int NODES_TO_DELETE_SIZE = 200000;
  std::vector<SOKOBAN_NODE*> nodesToDelete;
  nodesToDelete.reserve((int) (NODES_TO_DELETE_SIZE));

  closed.reserve(10000000);
  closed.max_load_factor(0.9);

  std::vector<SOKOBAN_NODE*> nodes;
  NodeManager nodeManager(NODES_TO_DELETE_SIZE, 524288);

  SOKOBAN_NODE* initialNode = makeRootNode();

  int heuristicaInicial = initialNode->heuristica;

  //Verbose stuff
  int lowestH = SHRT_MAX;
  int highestF = 0;
  int stateExpandDelay = 1000;
  long long int repeatedNodes = 0;

  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);

  if(initialNode->heuristica < SHRT_MAX){
    open.push(initialNode);
    addedToOpen++;
  }

  while(!open.empty()){
    SOKOBAN_NODE* n = open.top();
    open.pop();

    if(verbose && n->heuristica < lowestH){
      lowestH = n->heuristica;
      printf("Lowest Heuristic: %d\n", lowestH);
    }
    if(verbose && n->pathCost + n->heuristica > highestF){
      highestF = n->pathCost + n->heuristica;
      printf("Highest F-Value: %d (BF: %.2f)\n", highestF, ((float) open.size() / prevOpenSize));
      prevOpenSize = open.size();
    }

    bool keepNode = false;
    if(closed.find(*(n->state)) == closed.end()){
      closed.insert(*(n->state));

      if(isGoalState(*(n->state)))
        return extractPath(n, nodes, expanded, addedToOpen);

      expanded++;

      GlobalMemoryStatusEx(&statex);

      if(statex.dwMemoryLoad >= static_cast<unsigned int>(this->ramLimit)){
        if(verbose){
          printf("Out of memory! Terminating...\n");
        }

        if(initialNode->state != nullptr){
          delete initialNode->state;
          delete initialNode;
        }

        SOKOBAN_SOLUTION nosol;
        nosol.path.clear();
        nosol.cost = NO_RAM;
        nosol.expanded = expanded;
        nosol.heuristicaInicial = heuristicaInicial;
        nosol.addedToOpen = addedToOpen;
        nosol.timeMilis = 0;

        return nosol;
      }

      if(verbose && expanded % stateExpandDelay == 0){
        auto expandTestFinal = std::chrono::steady_clock::now();
        long long int expandTestTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(expandTestFinal - expandTestInicial).count();
        printf("Expanded: %lld States (Mem: %ld / %d) ", expanded, statex.dwMemoryLoad, this->ramLimit);
        printf("Open: %lld, Nodes: %ld, Closed: %lld, Repeated: %lld\n", (long long int) open.size(), nodeManager.size(), (long long int) closed.size(), repeatedNodes);
        if(expandTestTotal < 3000)
          stateExpandDelay = stateExpandDelay*10;
        if(expandTestTotal > 30000)
          stateExpandDelay = stateExpandDelay/2;
        expandTestInicial = expandTestFinal;
      }

      int minAddF = 9999;
      for(std::pair<ACTION, SOKOBAN_STATE> s : getSucc(*(n->state))){
        if(closed.find(s.second) == closed.end()){
          if(this->lowMemory && !movedBox(s.second, s.first) && !checkMoveCloser(*(n->state), s.first)){
            closed.insert(s.second);
          }
          else{const int fH = fHeuristica(s.second);

            if(fH < SHRT_MAX){
              if(n->pathCost + 1 + fH >= n->pathCost + n->heuristica + n->additionalF){
              
                if(n->pathCost + 1 + fH == n->pathCost + n->heuristica + n->additionalF){
                  SOKOBAN_NODE* nl = nodeManager.makeNode(n, s.first, s.second, fH);
                  open.push(nl);
                  addedToOpen++;
                }
                else{
                  if(n->pathCost + 1 + fH < n->pathCost + n->heuristica + n->additionalF + minAddF){
                    minAddF = (n->pathCost + 1 + fH) - n->pathCost - n->heuristica - n->additionalF;
                  }
                }
              }
            }
          }
        }
      }

      if(minAddF < 9999){
        keepNode = true;
        n->additionalF = n->additionalF + minAddF;
        open.push(n);
        closed.erase(*(n->state));
        addedToOpen++;
      }
      else{
        delete n->state;
        n->state = nullptr;
      }
    }
    
    if(n->nSuccs == 0 && !keepNode){
      nodesToDelete.push_back(n);
      if(nodesToDelete.size() == NODES_TO_DELETE_SIZE){
        nodeManager.deleteNodes(nodesToDelete);
      }
      repeatedNodes++;
    }
  }

  if(initialNode->state != nullptr){
    delete initialNode->state;
    delete initialNode;
  }

  SOKOBAN_SOLUTION nosol;
  nosol.path.clear();
  nosol.cost = NO_SOLUTION;
  nosol.expanded = expanded;
  nosol.heuristicaInicial = heuristicaInicial;
  nosol.addedToOpen = addedToOpen;
  nosol.timeMilis = 0;

  return nosol;
}

SOKOBAN_SOLUTION Sokoban::solveGBFS(bool verbose){
  auto expandTestInicial = std::chrono::steady_clock::now();

  long long int expanded = 0;
  long long int addedToOpen = 0;
  std::priority_queue<SOKOBAN_NODE*, std::vector<SOKOBAN_NODE*>, ComparePriorityGreedy> open;

  std::unordered_set<SOKOBAN_STATE> closed;
  closed.reserve(6000000);
  closed.max_load_factor(0.9);

  std::vector<SOKOBAN_NODE*> nodes;

  SOKOBAN_NODE* initialNode = makeRootNode();
  nodes.push_back(initialNode);

  int heuristicaInicial = initialNode->heuristica;

  //Verbose stuff
  int lowestH = SHRT_MAX;
  int highestF = 0;
  int stateExpandDelay = 1000;

  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);

  if(initialNode->heuristica < SHRT_MAX){
    open.push(initialNode);
    addedToOpen++;
  }

  while(!open.empty()){
    SOKOBAN_NODE* n = open.top();
    open.pop();

    if(verbose && n->heuristica < lowestH){
      lowestH = n->heuristica;
      printf("Lowest Heuristic: %d\n", lowestH);
    }
    if(verbose && n->pathCost + n->heuristica > highestF){
      highestF = n->pathCost + n->heuristica;
      printf("Highest F-Value: %d\n", highestF);
    }

    if(closed.find(*(n->state)) == closed.end()){
      closed.insert(*(n->state));

      if(isGoalState(*(n->state)))
        return extractPath(n, nodes, expanded, addedToOpen);

      expanded++;

      GlobalMemoryStatusEx (&statex);

      if(statex.dwMemoryLoad >= static_cast<unsigned int>(this->ramLimit)){
        if(verbose){
          printf("Out of memory! Terminating...\n");
        }

        for(SOKOBAN_NODE* n : nodes){
          delete n->state;
          delete n;
        }

        SOKOBAN_SOLUTION nosol;
        nosol.path.clear();
        nosol.cost = NO_RAM;
        nosol.expanded = expanded;
        nosol.heuristicaInicial = heuristicaInicial;
        nosol.addedToOpen = addedToOpen;
        nosol.timeMilis = 0;

        return nosol;
      }

      if(verbose && expanded % stateExpandDelay == 0){
        auto expandTestFinal = std::chrono::steady_clock::now();
        long long int expandTestTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(expandTestFinal - expandTestInicial).count();
        printf("Expanded: %lld States (Mem: %ld / %d)\n", expanded, statex.dwMemoryLoad, this->ramLimit);
        if(expandTestTotal < 3000)
          stateExpandDelay = stateExpandDelay*10;
        if(expandTestTotal > 30000)
          stateExpandDelay = stateExpandDelay/2;
        expandTestInicial = expandTestFinal;
      }

      for(std::pair<ACTION, SOKOBAN_STATE> s : getSucc(*(n->state))){
        if(closed.find(s.second) == closed.end()){
          SOKOBAN_NODE* nl = makeNode(n, s.first, s.second);
          if(nl->heuristica < SHRT_MAX){
            nodes.push_back(nl);
            open.push(nl);
            addedToOpen++;
          }
          else{
            delete nl->state;
            delete nl;
          }
        }
      }

      delete n->state;
      n->state = nullptr;
    }
  }

  for(SOKOBAN_NODE* n : nodes){
    delete n->state;
    delete n;
  }

  SOKOBAN_SOLUTION nosol;
  nosol.path.clear();
  nosol.cost = NO_SOLUTION;
  nosol.expanded = expanded;
  nosol.heuristicaInicial = heuristicaInicial;
  nosol.addedToOpen = addedToOpen;
  nosol.timeMilis = 0;

  if(!open.empty())
    nosol.cost = NO_RAM;

  return nosol;
}

std::vector<std::pair<ACTION, SOKOBAN_STATE>> Sokoban::getSucc(SOKOBAN_STATE &state){
  std::vector<std::pair<ACTION, SOKOBAN_STATE>> sucs;
  std::pair<ACTION, SOKOBAN_STATE> suc;
  POSITION p;
  SOKOBAN_STATE s;

  p = state.playerPosition - this->mapLenX;
  if(p >= 0 && gameMap[p] != WALL){
    auto bp = state.boxPositions.find(p);
    if(bp == state.boxPositions.end()){
      s.boxPositions = state.boxPositions;
      s.playerPosition = p;
      suc.first = UP;
      suc.second = s;
      sucs.push_back(suc);
    }
    else{
      p = p - this->mapLenX;
      if(p >= 0 && gameMap[p] != WALL){
        if(state.boxPositions.find(p) == state.boxPositions.end() && !this->isDeadEnd[p]){
          p = p + this->mapLenX;
          s.boxPositions = state.boxPositions;
          s.boxPositions.erase(s.boxPositions.find(p));
          s.playerPosition = p;
          p = p - this->mapLenX;
          s.boxPositions.insert(p);
          suc.first = UP;
          suc.second = s;

          if(!checkFrozen(suc.second, p))
            sucs.push_back(suc);
        }
      }
    }
  }

  p = state.playerPosition + this->mapLenX;
  if(p < this->mapLenX*this->mapLenY && gameMap[p] != WALL){
    auto bp = state.boxPositions.find(p);
    if(bp == state.boxPositions.end()){
      s.boxPositions = state.boxPositions;
      s.playerPosition = p;
      suc.first = DOWN;
      suc.second = s;
      sucs.push_back(suc);
    }
    else{
      p = p + this->mapLenX;
      if(p < this->mapLenX*this->mapLenY && gameMap[p] != WALL){
        if(state.boxPositions.find(p) == state.boxPositions.end() && !this->isDeadEnd[p]){
          p = p - this->mapLenX;
          s.boxPositions = state.boxPositions;
          s.boxPositions.erase(s.boxPositions.find(p));
          s.playerPosition = p;
          p = p + this->mapLenX;
          s.boxPositions.insert(p);
          suc.first = DOWN;
          suc.second = s;

          if(!checkFrozen(suc.second, p))
            sucs.push_back(suc);
        }
      }
    }
  }


  if(state.playerPosition % this->mapLenX != 0){
    p = state.playerPosition - 1;
    if(gameMap[p] != WALL){
      auto bp = state.boxPositions.find(p);
      if(bp == state.boxPositions.end()){
        s.boxPositions = state.boxPositions;
        s.playerPosition = p;
        suc.first = LEFT;
        suc.second = s;
        sucs.push_back(suc);
      }
      else{
        if(p % this->mapLenX != 0){
          p--;
          if(gameMap[p] != WALL){
            if(state.boxPositions.find(p) == state.boxPositions.end() && !this->isDeadEnd[p]){
              p++;
              s.boxPositions = state.boxPositions;
              s.boxPositions.erase(s.boxPositions.find(p));
              s.playerPosition = p;
              p--;
              s.boxPositions.insert(p);
              suc.first = LEFT;
              suc.second = s;

              if(!checkFrozen(suc.second, p))
                sucs.push_back(suc);
            }
          }
        }
      }
    }
  }

  if((state.playerPosition+1) % this->mapLenX != 0){
    p = state.playerPosition + 1;
    if(gameMap[p] != WALL){
      auto bp = state.boxPositions.find(p);
      if(bp == state.boxPositions.end()){
        s.boxPositions = state.boxPositions;
        s.playerPosition = p;
        suc.first = RIGHT;
        suc.second = s;
        sucs.push_back(suc);
      }
      else{
        if((p+1) % this->mapLenX != 0){
          p++;
          if(gameMap[p] != WALL){
            if(state.boxPositions.find(p) == state.boxPositions.end() && !this->isDeadEnd[p]){
              p--;
              s.boxPositions = state.boxPositions;
              s.boxPositions.erase(s.boxPositions.find(p));
              s.playerPosition = p;
              p++;
              s.boxPositions.insert(p);
              suc.first = RIGHT;
              suc.second = s;

              if(!checkFrozen(suc.second, p))
                sucs.push_back(suc);
            }
          }
        }
      }
    }
  }

  return sucs;
}

bool Sokoban::isGoalState(SOKOBAN_STATE &state){
  for(POSITION p : state.boxPositions){
    if(!this->isGoal[p])
      return false;
  }
  return true;
}

SOKOBAN_NODE* Sokoban::makeRootNode(){
  SOKOBAN_STATE* initialState = new SOKOBAN_STATE;
  POSITION p;

  for(int i = 0; i < this->mapLenY; i++){
    for(int j = 0; j < this->mapLenX; j++){
      p = i*this->mapLenX + j;

      if(this->gameMap[p] == PLAYER || this->gameMap[p] == PLAYER_AND_GOAL)
        initialState->playerPosition = p;

      if(this->gameMap[p] == BOX || this->gameMap[p] == GOAL_AND_BOX)
        initialState->boxPositions.insert(p);
    }
  }

  SOKOBAN_NODE* node = new SOKOBAN_NODE;
  node->parent = NULL;
  node->action = UP;
  node->state = initialState;
  node->heuristica = fHeuristica(*(node->state));
  node->pathCost = 0;
  node->additionalF = 0;
  node->nSuccs = 0;
  return node;
}

SOKOBAN_NODE* Sokoban::makeNode(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state){
  SOKOBAN_NODE* node = new SOKOBAN_NODE;
  node->parent = prt;
  node->action = action;
  node->state = new SOKOBAN_STATE;
  *(node->state) = state;
  node->heuristica = fHeuristica(state);
  node->pathCost = node->parent->pathCost + 1;
  node->additionalF = 0;

  node->parent->nSuccs = node->parent->nSuccs + 1;
  node->nSuccs = 0;
  return node;
}

SOKOBAN_NODE* Sokoban::makeNodePreSearch(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state, POSITION goal, int gValue){
  SOKOBAN_NODE* node = new SOKOBAN_NODE;
  node->parent = prt;
  node->action = action;
  node->state = new SOKOBAN_STATE;
  *(node->state) = state;

  node->heuristica = 0;
  for(POSITION bp : state.boxPositions)
    node->heuristica = node->heuristica + getTileDistance(bp, goal);

  node->pathCost = gValue;
  return node;
}

//Aux Functions
bool Sokoban::checkFrozen(SOKOBAN_STATE &state, POSITION boxPosition){
  for(int j = 0; j < this->mapLenX * this->mapLenY; j++)
    this->treatWall[j] = false;

  return isFrozen(state, boxPosition, false, false);
}

bool Sokoban::isFrozen(SOKOBAN_STATE &state, POSITION boxPosition, bool xFrozen, bool yFrozen){
  POSITION p;

  if(!xFrozen){
    if(boxPosition % this->mapLenX != 0){
      p = boxPosition - 1;
      if(this->gameMap[p] == WALL)
        xFrozen = true;
    }
    if((boxPosition+1) % this->mapLenX != 0){
      p = boxPosition + 1;
      if(this->gameMap[p] == WALL)
        xFrozen = true;
    }
    if((boxPosition % this->mapLenX == 0 || this->isDeadEnd[boxPosition - 1]) && ((boxPosition+1) % this->mapLenX == 0 || this->isDeadEnd[boxPosition + 1]))
      xFrozen = true;
    if(boxPosition % this->mapLenX != 0 && state.boxPositions.find(boxPosition-1) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      xFrozen = treatWall[boxPosition-1] || isFrozen(state, boxPosition-1, true, false);
    }
    else if((boxPosition+1) % this->mapLenX != 0 && state.boxPositions.find(boxPosition+1) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      xFrozen = treatWall[boxPosition+1] || isFrozen(state, boxPosition+1, true, false);
    }
  }
  if(!yFrozen){
    if(boxPosition - this->mapLenX >= 0){
      p = boxPosition - this->mapLenX;
      if(this->gameMap[p] == WALL)
        yFrozen = true;
    }
    if(boxPosition + this->mapLenX < this->mapLenX * this->mapLenY){
      p = boxPosition + this->mapLenX;
      if(this->gameMap[p] == WALL)
        yFrozen = true;
    }
    if((boxPosition - this->mapLenX < 0 || this->isDeadEnd[boxPosition - this->mapLenX]) && (boxPosition + this->mapLenX >= this->mapLenX * this->mapLenY || this->isDeadEnd[boxPosition + this->mapLenX]))
      yFrozen = true;
    if(boxPosition - this->mapLenX >= 0 && state.boxPositions.find(boxPosition - this->mapLenX) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      yFrozen = treatWall[boxPosition - this->mapLenX] || isFrozen(state, boxPosition - this->mapLenX, false, true);
    }
    else if(boxPosition + this->mapLenX < this->mapLenX * this->mapLenY && state.boxPositions.find(boxPosition + this->mapLenX) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      yFrozen = treatWall[boxPosition + this->mapLenX] || isFrozen(state, boxPosition + this->mapLenX, false, true);
    }
  }

  return xFrozen && yFrozen && !this->isGoal[boxPosition];
}

bool Sokoban::movedBox(SOKOBAN_STATE &state, ACTION action){
  POSITION p = state.playerPosition;
  switch(action){
    case UP:
      return (state.boxPositions.find(p - this->mapLenX) != state.boxPositions.end());
    case DOWN:
      return (state.boxPositions.find(p + this->mapLenX) != state.boxPositions.end());
    case LEFT:
      return (p % this->mapLenX != 0 && state.boxPositions.find(p - 1) != state.boxPositions.end());
    case RIGHT:
      return ((p+1) % this->mapLenX != 0 && state.boxPositions.find(p + 1) != state.boxPositions.end());
  }
  return false;
}

ACTION Sokoban::opposite(ACTION action){
  ACTION result = UP;
  switch(action){
    case UP:
      result = DOWN;
      break;
    case DOWN:
      result = UP;
      break;
    case LEFT:
      result = RIGHT;
      break;
    case RIGHT:
      result = LEFT;
      break;
  }
  return result;
}

int Sokoban::getTileDistance(POSITION a, POSITION b){
  return this->tileDistances[a * this->mapLenX * this->mapLenY + b];
}

int Sokoban::getBoxTileDistance(POSITION box, POSITION goal){
  return this->boxTileDistances[box * this->mapLenX * this->mapLenY + goal];
}

bool Sokoban::checkMoveCloser(SOKOBAN_STATE &state, ACTION action){
  int pp = state.playerPosition;
  int newPP = pp;
  switch(action){
    case UP:
      newPP = pp - this->mapLenX;
      break;
    case DOWN:
      newPP = pp + this->mapLenX;
      break;
    case LEFT:
      newPP = pp - 1;
      break;
    case RIGHT:
      newPP = pp + 1;
      break;
  }

  POSITION neighboor;

  calcTileDistanceBoxes(state, pp, distBoxesPP);
  calcTileDistanceBoxes(state, newPP, distBoxesNPP);

  for(POSITION bp : state.boxPositions){
    if(bp % this->mapLenX != 0){
      neighboor = bp - 1;
      if(distBoxesNPP[neighboor] < distBoxesPP[neighboor])
        return true;
    }
    if((bp + 1) % this->mapLenX != 0){
      neighboor = bp + 1;
      if(distBoxesNPP[neighboor] < distBoxesPP[neighboor])
        return true;
    }
    if(bp - this->mapLenX >= 0){
      neighboor = bp - this->mapLenX;
      if(distBoxesNPP[neighboor] < distBoxesPP[neighboor])
        return true;
    }
    if(bp + this->mapLenX < this->mapLenX * this->mapLenY){
      neighboor = bp + this->mapLenX;
      if(distBoxesNPP[neighboor] < distBoxesPP[neighboor])
        return true;
    }
  }

  return false;
}

void Sokoban::calcTileDistanceBoxes(SOKOBAN_STATE &state, POSITION p1, int *distances){
  const int tiles = this->mapLenX * this->mapLenY;
  std::queue<std::pair<POSITION, int>> q;
  std::unordered_set<POSITION> closed;
  std::pair<POSITION, int> auxP;

  auxP.first = p1;
  auxP.second = 0;
  q.push(auxP);

  for(int i = 0; i < tiles; i++)
    distances[i] = SHRT_MAX;

  while(!q.empty()){
    std::pair<POSITION, int> p = q.front();
    q.pop();

    if(closed.find(p.first) == closed.end()){
      closed.insert(p.first);
      distances[p.first] = p.second;

      POSITION neighboor = p.first;
      auxP.second = p.second + 1;

      neighboor = neighboor - this->mapLenX;
      if(neighboor >= 0 && gameMap[neighboor] != WALL && state.boxPositions.find(neighboor) == state.boxPositions.end()){
        auxP.first = neighboor;
        q.push(auxP);
      }
      neighboor = neighboor + 2*this->mapLenX;
      if(neighboor < this->mapLenX*this->mapLenY && gameMap[neighboor] != WALL && state.boxPositions.find(neighboor) == state.boxPositions.end()){
        auxP.first = neighboor;
        q.push(auxP);
      }
      neighboor = neighboor - this->mapLenX;

      if(neighboor % this->mapLenX != 0){
        neighboor--;
        if(gameMap[neighboor] != WALL && state.boxPositions.find(neighboor) == state.boxPositions.end()){
          auxP.first = neighboor;
          q.push(auxP);
        }
        neighboor++;
      }
      if((neighboor+1) % this->mapLenX != 0){
        neighboor++;
        if(gameMap[neighboor] != WALL && state.boxPositions.find(neighboor) == state.boxPositions.end()){
          auxP.first = neighboor;
          q.push(auxP);
        }
      }
    }
  }
}

void Sokoban::calculateTileDistances(){
  const int tiles = this->mapLenX * this->mapLenY;

  int *tempTileDistances = new int[tiles];
  std::queue<std::pair<POSITION, int>> q;
  std::unordered_set<POSITION> closed;
  std::pair<POSITION, int> auxP;

  for(POSITION i = 0; i < tiles; i++){
    for(int j = 0; j < tiles; j++)
      tempTileDistances[j] = SHRT_MAX;

    while(!q.empty()) q.pop();
    closed.clear();

    if(this->gameMap[i] != WALL){
      auxP.first = i;
      auxP.second = 0;
      q.push(auxP);
    }

    while(!q.empty()){
      std::pair<POSITION, int> p = q.front();
      q.pop();

      if(closed.find(p.first) == closed.end()){
        closed.insert(p.first);
        tempTileDistances[p.first] = p.second;

        POSITION neighboor = p.first;
        auxP.second = p.second + 1;

        neighboor = neighboor - this->mapLenX;
        if(neighboor >= 0 && gameMap[neighboor] != WALL){
          auxP.first = neighboor;
          q.push(auxP);
        }
        neighboor = neighboor + 2*this->mapLenX;
        if(neighboor < this->mapLenX*this->mapLenY && gameMap[neighboor] != WALL){
          auxP.first = neighboor;
          q.push(auxP);
        }
        neighboor = neighboor - this->mapLenX;

        if(neighboor % this->mapLenX != 0){
          neighboor--;
          if(gameMap[neighboor] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
          neighboor++;
        }
        if((neighboor+1) % this->mapLenX != 0){
          neighboor++;
          if(gameMap[neighboor] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
        }
      }
    }

    for(int j = 0; j < tiles; j++)
      this->tileDistances[i * tiles + j] = tempTileDistances[j];

  }
  delete[] tempTileDistances;
}

void Sokoban::calculateBoxTileDistances(){
  const int tiles = this->mapLenX * this->mapLenY;

  std::vector<SOKOBAN_NODE*> nodes;

  int playerStartPosition;
  int distance;

  for(int i = 0; i < tiles * tiles; i++)
    this->boxTileDistances[i] = SHRT_MAX;

  for(POSITION p = 0; p < tiles; p++){
    if(this->gameMap[p] == WALL || this->isDeadEnd[p])
      continue;

    for(POSITION p2 = 0; p2 < tiles; p2++){
      if(this->gameMap[p2] == WALL)
        continue;

      for(int playerStart = 0; playerStart < 4; playerStart++){
        switch (playerStart) {
          case 0:
            playerStartPosition = p - this->mapLenX;
            if(playerStartPosition < 0 || this->gameMap[playerStartPosition] == WALL)
              continue;
            break;
          case 1:
            playerStartPosition = p + this->mapLenX;
            if(playerStartPosition > tiles || this->gameMap[playerStartPosition] == WALL)
              continue;
            break;
          case 2:
            playerStartPosition = p - 1;
            if(p % this->mapLenX == 0 || this->gameMap[playerStartPosition] == WALL)
              continue;
            break;
          case 3:
            playerStartPosition = p + 1;
            if((p+1) % this->mapLenX == 0 || this->gameMap[playerStartPosition] == WALL)
              continue;
            break;
        }

        std::priority_queue<SOKOBAN_NODE*, std::vector<SOKOBAN_NODE*>, ComparePriorityAstar> open;
        std::unordered_set<SOKOBAN_STATE> closed;
        nodes.clear();
        distance = SHRT_MAX;

        SOKOBAN_STATE initialState;
        initialState.playerPosition = playerStartPosition;
        initialState.boxPositions.insert(p);

        SOKOBAN_NODE* root = makeNodePreSearch(NULL, UP, initialState, p2, 0);
        nodes.push_back(root);
        open.push(root);

        while(!open.empty() && distance == SHRT_MAX){
          SOKOBAN_NODE* n = open.top();
          open.pop();

          if(closed.find(*(n->state)) == closed.end()){
            closed.insert(*(n->state));

            if(n->state->boxPositions.find(p2) != n->state->boxPositions.end())
              distance = n->pathCost;
            else{
              for(std::pair<ACTION, SOKOBAN_STATE> s : getSucc(*(n->state))){
                if(closed.find(s.second) == closed.end()){
                  SOKOBAN_NODE* nl = makeNodePreSearch(n, s.first, s.second, p2, n->pathCost + 1);
                  if(nl->heuristica < SHRT_MAX){
                    nodes.push_back(nl);
                    open.push(nl);
                  }
                  else{
                    delete nl->state;
                    delete nl;
                  }
                }
              }

              delete n->state;
              n->state = nullptr;
            }
          }
        }

        for(SOKOBAN_NODE* np : nodes){
          if(np->state != nullptr)
            delete np->state;
          delete np;
        }

        if(distance < this->boxTileDistances[p * tiles + p2])
          this->boxTileDistances[p * tiles + p2] = distance;
      }
    }
  }
}
