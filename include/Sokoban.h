#ifndef SOKOBAN_H
#define SOKOBAN_H

#define RAM_LIMIT 95
#define NO_SOLUTION -1
#define NO_RAM -2

#pragma pack(push, 1)

enum ACTION {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3};
enum GAME_TILE {EMPTY = ' ', WALL = '#', BOX = '$', GOAL = '.', PLAYER = '@', GOAL_AND_BOX = '*', PLAYER_AND_GOAL = '+'};

struct SOKOBAN_SOLUTION{
  int cost;
  std::vector<ACTION> path;

  long long int timeMilis;
  long long int expanded;
  long long int addedToOpen;

  int initialHeuristic;
};

struct SOKOBAN_STATE{
  BoxPositionSet boxPositions;
  POSITION playerPosition;

  bool operator==(const SOKOBAN_STATE &s2) const{

    if(playerPosition != s2.playerPosition)
      return false;

    ConstantSetIterator it1 = boxPositions.begin();
    ConstantSetIterator it2 = s2.boxPositions.begin();
    for(int i = 0; i < BoxPositionSet::STATE_BOX_QUANT; i++){
      if(*it1 != *it2)
        return false;
      it1++;
      it2++;
    }
    return true;
  }
};

struct TUNNEL_MACRO {
  POSITION player;
  POSITION box;

  bool operator==(const TUNNEL_MACRO &s2) const{
    return player == s2.player && box == s2.box;
  }
};

namespace std {
    template<> struct hash<SOKOBAN_STATE>
    {
        std::size_t operator()(const SOKOBAN_STATE& estado) const noexcept
        {
            size_t result = estado.playerPosition;
            int i = 0;
            for(auto x : estado.boxPositions){
              result = result ^ ((size_t) x << ((i%4 + 1)*16));
              i++;
            }
            return result;
        }
    };

    template<> struct hash<TUNNEL_MACRO>
    {
        std::size_t operator()(const TUNNEL_MACRO& tm) const noexcept
        {
            size_t result = static_cast<size_t>(tm.player) * 65536 + static_cast<size_t>(tm.box);    
            return result;
        }
    };
}

typedef struct soknode_info{
  short int pathCost;
  short int heuristic;
  short int additionalF;
  SOKOBAN_STATE state;
} NodeInfo;

typedef struct soknode{
  NodeInfo *info;
  soknode *parent;
  ACTION action;
  unsigned char nUseCount;
} SOKOBAN_NODE;

#pragma pack(pop)

class Sokoban{
  public:
    Sokoban(FILE* inputMap);
    ~Sokoban();

    SOKOBAN_SOLUTION solve(bool verbose, bool lowMemory, int ramLimit);
  private:
    int mapLenX, mapLenY, mapLenTotal;
    int mapBoxQuant;
    int ramLimit;
    bool lowMemory;

    char *gameMap;

    bool *isDeadEnd;

    bool *isGoal;

    int *minGoalDistance;

    int *tileDistances; //Distances between tiles (player)
    int *boxTileDistances; //Distances beween tiles (boxes)

    SOKOBAN_SOLUTION solvePEAstar(bool verbose);

    int fHeuristica(SOKOBAN_STATE &state);

    bool *treatWall;

    int *distBoxesPP;
    int *distBoxesNPP;

    std::unordered_map<TUNNEL_MACRO, ACTION> tunnelMacros;

    void getSucc(SOKOBAN_STATE& state, SOKOBAN_STATE *succs, bool *valids);
    bool isGoalState(SOKOBAN_STATE &state);
    SOKOBAN_NODE* makeRootNode();
    SOKOBAN_NODE* makeNodePreSearch(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state, POSITION goal, int gValue);
    SOKOBAN_SOLUTION extractPath(SOKOBAN_NODE* n, std::vector<SOKOBAN_NODE*> &nodes, long long int expanded, long long int addedToOpen);

    void shrinkOpen(PriQueue* open, StateSet* closed, std::vector<SOKOBAN_NODE*>& nodesToDelete, const bool verbose);

    //Aux Functions
    bool checkFrozen(SOKOBAN_STATE &state, POSITION boxPosition);
    bool isFrozen(SOKOBAN_STATE &state, POSITION boxPosition, bool xFrozen, bool yFrozen);
    bool movedBox(SOKOBAN_STATE &state, ACTION action);
    ACTION opposite(ACTION action);
    int getTileDistance(POSITION a, POSITION b);
    int getBoxTileDistance(POSITION box, POSITION goal);
    bool checkMoveCloser(SOKOBAN_STATE &state, ACTION action);
    void calcTileDistanceBoxes(SOKOBAN_STATE &state, POSITION p1, int *distances);
    void calculateTileDistances();
    void calculateBoxTileDistances();
    void calculateTunnelMacros();
};

#endif