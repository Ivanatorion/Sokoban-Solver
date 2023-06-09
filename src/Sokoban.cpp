#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/PriQueue.h"
#include "../include/NodeManager.h"
#include "../include/StateSet.h"

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
  this->mapLenTotal = this->mapLenX * this->mapLenY;

  this->gameMap = new char[this->mapLenTotal];
  this->isDeadEnd = new bool[this->mapLenTotal];
  this->isGoal = new bool[this->mapLenTotal];
  this->minGoalDistance = new int[this->mapLenTotal];

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

  this->treatWall = new bool[this->mapLenTotal];
  this->distBoxesPP = new int[this->mapLenTotal];
  this->distBoxesNPP = new int[this->mapLenTotal];

  std::queue<std::pair<POSITION, int>> q;
  std::unordered_set<POSITION> closed;
  for(POSITION mapPos = 0; mapPos < this->mapLenTotal; mapPos++){
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

  calculateTunnelMacros();

  //Calculate Tile Distances
  this->tileDistances = new int[this->mapLenTotal * this->mapLenTotal];
  calculateTileDistances();
  this->boxTileDistances = new int[this->mapLenTotal * this->mapLenTotal];
  calculateBoxTileDistances();

  //Updates minGoalDistance
  std::vector<POSITION> goalPositions;
  for(POSITION p = 0; p < this->mapLenTotal; p++){
    if(this->isGoal[p])
      goalPositions.push_back(p);
  }

  for(POSITION p = 0; p < this->mapLenTotal; p++){
    this->minGoalDistance[p] = SHRT_MAX;
    for(POSITION gp : goalPositions)
      if(getBoxTileDistance(p, gp) < this->minGoalDistance[p])
        this->minGoalDistance[p] = getBoxTileDistance(p, gp);
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
  printf("\nTunnel Macros:\n");
  for(auto it : tunnelMacros){
    printf("(%d %d) - (%d %d): ", it.first.player % this->mapLenX, it.first.player / this->mapLenX, it.first.box % this->mapLenX, it.first.box / this->mapLenX);
    switch(it.second){
      case UP:
        printf("UP\n");
        break;
      case DOWN:
        printf("DOWN\n");
        break;
      case LEFT:
        printf("LEFT\n");
        break;
      case RIGHT:
        printf("RIGHT\n");
        break;
    }
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

  delete[] this->treatWall;

  delete[] this->distBoxesPP;
  delete[] this->distBoxesNPP;
}


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
    if(border < this->mapLenTotal){
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
}

SOKOBAN_SOLUTION Sokoban::extractPath(SOKOBAN_NODE* n, std::vector<SOKOBAN_NODE*> &nodes, long long int expanded, long long int addedToOpen){
  SOKOBAN_SOLUTION sol;
  sol.cost = n->info->pathCost;
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
    if(np->info != nullptr)
      delete np->info;

    delete np;
  }

  sol.initialHeuristic = -1;

  return sol;
}

SOKOBAN_SOLUTION Sokoban::solve(bool verbose, bool lowMemory, int ramLimit){
  auto tempoInicial = std::chrono::steady_clock::now();

  SOKOBAN_SOLUTION solution;

  this->lowMemory = lowMemory;

  if(ramLimit >= 1 && ramLimit <= 98)
    this->ramLimit = ramLimit;
  
  solution = solvePEAstar(verbose);

  auto tempoFinal = std::chrono::steady_clock::now();
  long long int tempoTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(tempoFinal - tempoInicial).count();
  solution.timeMilis = tempoTotal;

  return solution;
}

void Sokoban::shrinkOpen(PriQueue* open, StateSet* closed, std::vector<SOKOBAN_NODE*>& nodesToDelete, const bool verbose){
  const int before = (int) open->size();

  if(verbose)
    printf("Shrink Open Before: %d\n", before);
  SOKOBAN_NODE** tempN = (SOKOBAN_NODE**) malloc(sizeof(SOKOBAN_NODE*) * (int) open->size());
    
  int curN = 0;
  while(!open->empty()){
	  SOKOBAN_NODE* n = open->deleteMin();

	  if(!closed->isInSet(n->info->state)){
	    tempN[curN] = n;
	    curN++;
	  }
    else{
      if(n->nUseCount == 0){
        nodesToDelete.push_back(n);
      }
    }
  }

  for(int i = 0; i < curN; i++)
	  open->push(tempN[i], (tempN[i]->info->pathCost + tempN[i]->info->heuristic + tempN[i]->info->additionalF) * (10000) + tempN[i]->info->heuristic);

  free(tempN);
  const int after = (int) open->size();

  if(verbose)
    printf("Shrink Open After: %d (%.2f%% nodes deleted)\n", after, 100.0 - (100.0 * after) / before);
}

SOKOBAN_SOLUTION Sokoban::solvePEAstar(bool verbose){
  auto expandTestInicial = std::chrono::steady_clock::now();

  long long int expanded = 0;
  long long int addedToOpen = 0;
  long long int prevOpenSize = 1;

  int closedShrinkCount = 0;
  long long int closedSizeOnShrink = 0;
  bool closedShrinkStop = false;
  
  PriQueue open(8);
  StateSet closed(BoxPositionSet::STATE_BOX_QUANT);

  const int NODES_TO_DELETE_SIZE = 200000;
  std::vector<SOKOBAN_NODE*> nodesToDelete;
  nodesToDelete.reserve((int) (NODES_TO_DELETE_SIZE));

  std::vector<SOKOBAN_NODE*> nodes;
  NodeManager nodeManager(524288);

  SOKOBAN_NODE* initialNode = makeRootNode();

  int initialHeuristic = initialNode->info->heuristic;

  //Verbose stuff
  int lowestH = SHRT_MAX;
  int highestF = 0;
  int stateExpandDelay = 1000;

  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);
  GlobalMemoryStatusEx(&statex);
  const DWORD ClosedShrinkLoad = (((this->ramLimit - statex.dwMemoryLoad) * 17) / 30) + statex.dwMemoryLoad;
  if(verbose)
    printf("ClosedShrikLoad: %d\n", (int) ClosedShrinkLoad);

  if(initialNode->info->heuristic < SHRT_MAX){
    open.push(initialNode, initialNode->info->pathCost + initialNode->info->heuristic + initialNode->info->additionalF);
    addedToOpen++;
  }

  SOKOBAN_STATE succVec[4];
  bool succVecValid[4];

  while(!open.empty()){
    SOKOBAN_NODE* n = open.deleteMin();

    if(verbose && n->info->heuristic < lowestH){
      lowestH = n->info->heuristic;
      printf("Lowest Heuristic: %d\n", lowestH);
    }
    if(verbose && n->info->pathCost + n->info->heuristic > highestF){
      highestF = n->info->pathCost + n->info->heuristic;
      printf("Highest F-Value: %d (BF: %.2f)\n", highestF, ((float) open.size() / prevOpenSize));
      prevOpenSize = open.size();
    }

    if(!closed.isInSet(n->info->state)){
      closed.insert(n->info->state);

      if(isGoalState(n->info->state)){
        /*
        if(initialNode->info != nullptr){
          delete initialNode->info;
        }
        delete initialNode;
        */
        return extractPath(n, nodes, expanded, addedToOpen);
      }

      expanded++;

      GlobalMemoryStatusEx(&statex);

      if(statex.dwMemoryLoad >= static_cast<unsigned int>(this->ramLimit)){
        if(verbose)
          printf("Out of memory! Terminating...\n");

        if(initialNode->info != nullptr){
          delete initialNode->info;
          delete initialNode;
        }

        SOKOBAN_SOLUTION nosol;
        nosol.path.clear();
        nosol.cost = NO_RAM;
        nosol.expanded = expanded;
        nosol.initialHeuristic = initialHeuristic;
        nosol.addedToOpen = addedToOpen;
        nosol.timeMilis = 0;

        return nosol;
      }

      if(expanded % stateExpandDelay == 0){
        
        auto expandTestFinal = std::chrono::steady_clock::now();
        long long int expandTestTotal = (long long int) std::chrono::duration_cast<std::chrono::milliseconds>(expandTestFinal - expandTestInicial).count();
        if(verbose){
          printf("Expanded: %lld States (Mem: %ld / %d) ", expanded, statex.dwMemoryLoad, this->ramLimit);
          printf("Open: %lld, Nodes: %ld, Deleted: %ld, Closed: %lld\n", (long long int) open.size(), nodeManager.size(), nodeManager.deletedQuant(), (long long int) closed.size());
        }
        if(expandTestTotal < 3000)
          stateExpandDelay = stateExpandDelay*10;
        if(expandTestTotal > 30000)
          stateExpandDelay = stateExpandDelay/2;

        if(closedShrinkCount < 1 && statex.dwMemoryLoad >= ClosedShrinkLoad){
          closedShrinkCount++;
          closedSizeOnShrink = (closed.size() * 96) / 100;
          this->shrinkOpen(&open, &closed, nodesToDelete, verbose);
          closed.shrink(gameMap, this->mapLenX, this->mapLenY, verbose);
          if(verbose)
            printf("Next ClosedSizeShrink: %lld\n", closedSizeOnShrink);
        }
        else if(closedShrinkCount > 0 && closed.size() >= closedSizeOnShrink && !closedShrinkStop){
          closedShrinkCount++;
          closedSizeOnShrink = (closed.size() * 96) / 100;
          this->shrinkOpen(&open, &closed, nodesToDelete, verbose);
          
          long long int before = closed.size();
          closed.shrink(gameMap, this->mapLenX, this->mapLenY, verbose);
          long long int after = closed.size();
          if((before - after) * 10 < before){
            closedShrinkStop = true;
            if(verbose)
              printf("Last shrink...\n");
          }
          else if(verbose)
            printf("Next ClosedSizeShrink: %lld\n", closedSizeOnShrink);
        }

        expandTestInicial = std::chrono::steady_clock::now();
      }

      int minAddF = 9999;

      bool tunnelMacro = false;
      ACTION tunnelMacroAction = UP;
      if(n != initialNode){
        std::vector<POSITION> tmBoxPositions = {static_cast<POSITION>(n->info->state.playerPosition - this->mapLenX), static_cast<POSITION>(n->info->state.playerPosition + this->mapLenX),  static_cast<POSITION>(n->info->state.playerPosition - 1),  static_cast<POSITION>(n->info->state.playerPosition + 1)};
        for(int i = 0; i < 4; i++){
          if(n->info->state.boxPositions.find(tmBoxPositions[i]) != n->info->state.boxPositions.end()){
            auto itTM = this->tunnelMacros.find(TUNNEL_MACRO{n->info->state.playerPosition, tmBoxPositions[i]});
            if(itTM != this->tunnelMacros.end()){
              tunnelMacro = true;
              tunnelMacroAction = itTM->second;
            }
          }
        }
      }

      getSucc(n->info->state, succVec, succVecValid);
      for(int i = 0; i < 4; i++){
        if(succVecValid[i] && !closed.isInSet(succVec[i]) && (!tunnelMacro || tunnelMacroAction == static_cast<ACTION>(i))){
          if(this->lowMemory && !movedBox(succVec[i], static_cast<ACTION>(i)) && !checkMoveCloser(n->info->state, static_cast<ACTION>(i))){
            closed.insert(succVec[i]);
          }
          else{
            const int fH = fHeuristica(succVec[i]);
            if(fH < SHRT_MAX){
              if(n->info->pathCost + 1 + fH >= n->info->pathCost + n->info->heuristic + n->info->additionalF){
                if(n->info->pathCost + 1 + fH == n->info->pathCost + n->info->heuristic + n->info->additionalF){
                  SOKOBAN_NODE* nl = nodeManager.makeNode(n, static_cast<ACTION>(i), succVec[i], fH);
                  open.push(nl, (nl->info->pathCost + nl->info->heuristic + nl->info->additionalF) * (10000) + nl->info->heuristic);
                  addedToOpen++;
                }
                else{
                  if(n->info->pathCost + 1 + fH < n->info->pathCost + n->info->heuristic + n->info->additionalF + minAddF){
                    minAddF = (n->info->pathCost + 1 + fH) - n->info->pathCost - n->info->heuristic - n->info->additionalF;
                  }
                }
              }
            }
          }
        }
      }

      if(minAddF < 9999){
        n->info->additionalF = n->info->additionalF + minAddF;
        open.push(n, (n->info->pathCost + n->info->heuristic + n->info->additionalF) * (10000) + n->info->heuristic);
        closed.erase(n->info->state);
        addedToOpen++;
      }
      else{
        delete n->info;
        n->info = nullptr;
      }
    }
    
    if(n->nUseCount == 0){
      nodesToDelete.push_back(n);
      if(nodesToDelete.size() >= NODES_TO_DELETE_SIZE){
        nodeManager.deleteNodes(nodesToDelete);
        nodesToDelete.clear();
        nodesToDelete.reserve(NODES_TO_DELETE_SIZE);
      }
    }
  }

  if(initialNode->info != nullptr){
    delete initialNode->info;
    delete initialNode;
  }

  SOKOBAN_SOLUTION nosol;
  nosol.path.clear();
  nosol.cost = NO_SOLUTION;
  nosol.expanded = expanded;
  nosol.initialHeuristic = initialHeuristic;
  nosol.addedToOpen = addedToOpen;
  nosol.timeMilis = 0;

  return nosol;
}

void Sokoban::getSucc(SOKOBAN_STATE& state, SOKOBAN_STATE *succs, bool *valids){
  POSITION p;

  for(int i = 0; i < 4; i++)
    valids[i] = false;

  //UP
  p = state.playerPosition - this->mapLenX;
  if(gameMap[p] != WALL){
    if(state.boxPositions.find(p) == state.boxPositions.end()){
      succs[0].boxPositions = state.boxPositions;
      succs[0].playerPosition = p;
      valids[0] = true;
    }
    else{
      p = p - this->mapLenX;
      if(gameMap[p] != WALL && !this->isDeadEnd[p] && state.boxPositions.find(p) == state.boxPositions.end()){
        p = p + this->mapLenX;
        succs[0].boxPositions = state.boxPositions;
        succs[0].boxPositions.replace(p, p - this->mapLenX);
        succs[0].playerPosition = p;
        p = p - this->mapLenX;

        if(!checkFrozen(succs[0], p)){
          valids[0] = true;
        }
      }
    }
  }

  //DOWN
  p = state.playerPosition + this->mapLenX;
  if(gameMap[p] != WALL){
    if(state.boxPositions.find(p) == state.boxPositions.end()){
      succs[1].boxPositions = state.boxPositions;
      succs[1].playerPosition = p;
      valids[1] = true;
    }
    else{
      p = p + this->mapLenX;
      if(gameMap[p] != WALL && !this->isDeadEnd[p] && state.boxPositions.find(p) == state.boxPositions.end()){  
        p = p - this->mapLenX;
        succs[1].boxPositions = state.boxPositions;
        succs[1].boxPositions.replace(p, p + this->mapLenX);
        succs[1].playerPosition = p;
        p = p + this->mapLenX;

        if(!checkFrozen(succs[1], p)){
          valids[1] = true;
        }
      }
    }
  }

  //LEFT
  p = state.playerPosition - 1;
  if(gameMap[p] != WALL){
    if(state.boxPositions.find(p) == state.boxPositions.end()){
      succs[2].boxPositions = state.boxPositions;
      succs[2].playerPosition = p;
      valids[2] = true;
    }
    else{
      p--;
      if(gameMap[p] != WALL && !this->isDeadEnd[p] && state.boxPositions.find(p) == state.boxPositions.end()){
        p++;
        succs[2].boxPositions = state.boxPositions;
        succs[2].boxPositions.replace(p, p - 1);
        succs[2].playerPosition = p;
        p--;
        
        if(!checkFrozen(succs[2], p)){
          valids[2] = true;
        }
      }
    }
  }

  //RIGHT
  p = state.playerPosition + 1;
  if(gameMap[p] != WALL){
    if(state.boxPositions.find(p) == state.boxPositions.end()){
      succs[3].boxPositions = state.boxPositions;
      succs[3].playerPosition = p;
      valids[3] = true;
    }
    else{
      p++;
      if(gameMap[p] != WALL && !this->isDeadEnd[p] && state.boxPositions.find(p) == state.boxPositions.end()){
        p--;
        succs[3].boxPositions = state.boxPositions;
        succs[3].boxPositions.replace(p, p + 1);
        succs[3].playerPosition = p;
        p++;
        
        if(!checkFrozen(succs[3], p)){
          valids[3] = true;
        }
      }
    }
  }
}

bool Sokoban::isGoalState(SOKOBAN_STATE &state){
  for(POSITION p : state.boxPositions){
    if(!this->isGoal[p])
      return false;
  }
  return true;
}

SOKOBAN_NODE* Sokoban::makeRootNode(){
  SOKOBAN_STATE initialState;
  POSITION p;

  for(int i = 0; i < this->mapLenY; i++){
    for(int j = 0; j < this->mapLenX; j++){
      p = i*this->mapLenX + j;

      if(this->gameMap[p] == PLAYER || this->gameMap[p] == PLAYER_AND_GOAL)
        initialState.playerPosition = p;

      if(this->gameMap[p] == BOX || this->gameMap[p] == GOAL_AND_BOX)
        initialState.boxPositions.insert(p);
    }
  }

  SOKOBAN_NODE* node = new SOKOBAN_NODE;
  node->parent = NULL;
  node->action = UP;
  node->info = new NodeInfo;
  node->info->state = initialState;
  node->info->heuristic = fHeuristica(node->info->state);
  node->info->pathCost = 0;
  node->info->additionalF = 0;
  node->nUseCount = 0;
  return node;
}

SOKOBAN_NODE* Sokoban::makeNodePreSearch(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state, POSITION goal, int gValue){
  SOKOBAN_NODE* node = new SOKOBAN_NODE;
  node->parent = prt;
  node->action = action;
  node->info = new NodeInfo;
  node->info->state = state;

  node->info->heuristic = 0;
  for(POSITION bp : state.boxPositions)
    node->info->heuristic = node->info->heuristic + getTileDistance(bp, goal);

  node->info->pathCost = gValue;
  return node;
}

//Aux Functions
bool Sokoban::checkFrozen(SOKOBAN_STATE &state, POSITION boxPosition){
  for(int j = 0; j < this->mapLenTotal; j++)
    this->treatWall[j] = false;

  return isFrozen(state, boxPosition, false, false);
}

bool Sokoban::isFrozen(SOKOBAN_STATE &state, POSITION boxPosition, bool xFrozen, bool yFrozen){
  if(this->isGoal[boxPosition]) 
    return false;

  if(!xFrozen){
    if(this->gameMap[boxPosition - 1] == WALL || this->gameMap[boxPosition + 1] == WALL)
      xFrozen = true;
    else if(this->isDeadEnd[boxPosition - 1] && this->isDeadEnd[boxPosition + 1])
      xFrozen = true;
    if(state.boxPositions.find(boxPosition-1) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      xFrozen = treatWall[boxPosition-1] || isFrozen(state, boxPosition-1, true, false);
    }
    else if(state.boxPositions.find(boxPosition+1) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      xFrozen = treatWall[boxPosition+1] || isFrozen(state, boxPosition+1, true, false);
    }
  }
  if(!yFrozen){
    if(this->gameMap[boxPosition - this->mapLenX] == WALL || this->gameMap[boxPosition + this->mapLenX] == WALL)
      yFrozen = true;
    else if(this->isDeadEnd[boxPosition - this->mapLenX] && this->isDeadEnd[boxPosition + this->mapLenX])
      yFrozen = true;
    if(state.boxPositions.find(boxPosition - this->mapLenX) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      yFrozen = treatWall[boxPosition - this->mapLenX] || isFrozen(state, boxPosition - this->mapLenX, false, true);
    }
    else if(state.boxPositions.find(boxPosition + this->mapLenX) != state.boxPositions.end()){
      treatWall[boxPosition] = true;
      yFrozen = treatWall[boxPosition + this->mapLenX] || isFrozen(state, boxPosition + this->mapLenX, false, true);
    }
  }

  return xFrozen && yFrozen;
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
  return this->tileDistances[a * this->mapLenTotal + b];
}

int Sokoban::getBoxTileDistance(POSITION box, POSITION goal){
  return this->boxTileDistances[box * this->mapLenTotal + goal];
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
    if(bp + this->mapLenX < this->mapLenTotal){
      neighboor = bp + this->mapLenX;
      if(distBoxesNPP[neighboor] < distBoxesPP[neighboor])
        return true;
    }
  }

  return false;
}

void Sokoban::calcTileDistanceBoxes(SOKOBAN_STATE &state, POSITION p1, int *distances){
  const int tiles = this->mapLenTotal;
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
  const int tiles = this->mapLenTotal;

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
  const int tiles = this->mapLenTotal;

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
        if(neighboor >= 0 && gameMap[neighboor] != WALL && gameMap[neighboor + 2 * this->mapLenX] != WALL){
          auxP.first = neighboor;
          q.push(auxP);
        }
        neighboor = neighboor + 2 * this->mapLenX;
        if(neighboor < this->mapLenX*this->mapLenY && gameMap[neighboor] != WALL && gameMap[neighboor - 2 * this->mapLenX] != WALL){
          auxP.first = neighboor;
          q.push(auxP);
        }
        neighboor = neighboor - this->mapLenX;

        if(neighboor % this->mapLenX != 0){
          neighboor--;
          if(gameMap[neighboor] != WALL && gameMap[neighboor + 2] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
          neighboor++;
        }
        if((neighboor+1) % this->mapLenX != 0){
          neighboor++;
          if(gameMap[neighboor] != WALL && gameMap[neighboor - 2] != WALL){
            auxP.first = neighboor;
            q.push(auxP);
          }
        }
      }
    }

    for(int j = 0; j < tiles; j++)
      this->boxTileDistances[i * tiles + j] = tempTileDistances[j];

  }
  delete[] tempTileDistances;
}

void Sokoban::calculateTunnelMacros(){
  const int tiles = this->mapLenTotal;
  POSITION wall1, wall2, boxP1, boxP2, aux;
  tunnelMacros.clear();
  for(POSITION i = 0; i < (POSITION) tiles; i++){
    if(gameMap[i] == WALL || i % this->mapLenX == 0 || (i+1) % this->mapLenX == 0 || i < this->mapLenX || i + this->mapLenX >= this->mapLenTotal)
      continue;

    wall1 = i - this->mapLenX;
    wall2 = i + this->mapLenX;
    if(wall1 >= 0 && wall2 < this->mapLenTotal && gameMap[wall1] == WALL && gameMap[wall2] == WALL){
      boxP1 = i - 1;
      boxP2 = i + 1;
      if(boxP1 > 0 && gameMap[boxP1] != WALL && !this->isGoal[boxP1]){
        wall1 = boxP1 - this->mapLenX;
        wall2 = boxP1 + this->mapLenX;
        if((wall1 >= 0 && gameMap[wall1] == WALL) || (wall2 < this->mapLenTotal && gameMap[wall2] == WALL)){
          aux = boxP1 - 1;
          if(aux >= 0 && gameMap[aux] != WALL)
            tunnelMacros.emplace(TUNNEL_MACRO{i, boxP1}, LEFT);
        }
      }
      if(boxP2 < this->mapLenTotal && gameMap[boxP2] != WALL && !this->isGoal[boxP2]){
        wall1 = boxP2 - this->mapLenX;
        wall2 = boxP2 + this->mapLenX;
        if((wall1 >= 0 && gameMap[wall1] == WALL) || (wall2 < this->mapLenTotal && gameMap[wall2] == WALL)){
          aux = boxP2 + 1;
          if(aux < this->mapLenTotal && gameMap[aux] != WALL)
            tunnelMacros.emplace(TUNNEL_MACRO{i, boxP2}, RIGHT);
        }
      }
    }

    wall1 = i - 1;
    wall2 = i + 1;
    if(wall1 >= 0 && wall2 < this->mapLenTotal && gameMap[wall1] == WALL && gameMap[wall2] == WALL){
      boxP1 = i - this->mapLenX;
      boxP2 = i + this->mapLenX;
      if(boxP1 > 0 && gameMap[boxP1] != WALL && !this->isGoal[boxP1]){
        wall1 = boxP1 - 1;
        wall2 = boxP1 + 1;
        if((wall1 >= 0 && gameMap[wall1] == WALL) || (wall2 < this->mapLenTotal && gameMap[wall2] == WALL)){
          aux = boxP1 - this->mapLenX;
          if(aux >= 0 && gameMap[aux] != WALL)
            tunnelMacros.emplace(TUNNEL_MACRO{i, boxP1}, UP);
        }
      }
      if(boxP2 < this->mapLenTotal && gameMap[boxP2] != WALL && !this->isGoal[boxP2]){
        wall1 = boxP2 - 1;
        wall2 = boxP2 + 1;
        if((wall1 >= 0 && gameMap[wall1] == WALL) || (wall2 < this->mapLenTotal && gameMap[wall2] == WALL)){
          aux = boxP2 + this->mapLenX;
          if(aux < this->mapLenTotal && gameMap[aux] != WALL)
            tunnelMacros.emplace(TUNNEL_MACRO{i, boxP2}, DOWN);
        }
      }
    }
  }
}
