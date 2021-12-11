#include <cstdio>
#include <cmath>
#include <vector>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <set>

#define RAM_LIMIT 1000000000
#define NO_SOLUTION -1
#define NO_RAM -2

enum ACTION {UP, DOWN, LEFT, RIGHT};
enum GAME_TILE {EMPTY = ' ', WALL = '#', BOX = '$', GOAL = '.', PLAYER = '@', GOAL_AND_BOX = '*', PLAYER_AND_GOAL = '+'};

enum ALGO {ASTAR, GREEDY, IDASTAR, PEASTAR};

struct SOKOBAN_SOLUTION{
  int cost;
  std::vector<ACTION> path;

  long long int timeMilis;
  long long int expanded;
  long long int addedToOpen;
  long long int maximumOpenSize;

  int heuristicaInicial;
};

typedef short int POSITION;

struct SOKOBAN_STATE{
  POSITION playerPosition;
  std::set<POSITION> boxPositions;

  bool operator==(const SOKOBAN_STATE &s2) const{

    if(playerPosition == s2.playerPosition){
      for(POSITION p : boxPositions){
        if(s2.boxPositions.find(p) == s2.boxPositions.end())
          return false;
      }

      return true;
    }

    return false;
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
}

typedef struct soknode{
  short int pathCost;
  short int heuristica;
  SOKOBAN_STATE *state;
  soknode *parent;
  ACTION action;
  short int additionalF;
} SOKOBAN_NODE;

class Sokoban{
  public:
    Sokoban(FILE* inputMap);
    ~Sokoban();

    SOKOBAN_SOLUTION solve(bool verbose, bool lowMemory, bool greedy, ALGO algo, int idastarFlimit);
  private:
    int mapLenX, mapLenY;
    int mapBoxQuant;
    bool lowMemory;

    char *gameMap;

    bool *isDeadEnd;

    bool *isGoal;
    std::vector<POSITION> goalPositions;

    int *minGoalDistance;

    int *tileDistances; //Distances between tiles (player)
    int *boxTileDistances; //Distances beween tiles (boxes)

    SOKOBAN_SOLUTION solveAstar(bool verbose);
    SOKOBAN_SOLUTION solvePEAstar(bool verbose);
    SOKOBAN_SOLUTION solveGBFS(bool verbose);

    std::pair<int, std::vector<ACTION>> recursiveSearchIdastar(SOKOBAN_NODE* node, int fLimit, int *expanded, bool *noneSol, std::unordered_set<SOKOBAN_STATE> &transpositionTable);
    SOKOBAN_SOLUTION solveIdAstar(bool verbose, int idastarFlimit);

    int fHeuristica(SOKOBAN_STATE &state);
    bool forceAssing(int col);
    int *mmMatrix, *staticmmMatrix;
    int *goalAssigned;
    int *boxAssigned;
    bool *couldntAssignBox;
    bool *couldntAssignGoal;
    int *numberZeroesRow;
    int *numberZeroesCol;
    bool *tickedRows;
    bool *tickedCols;

    int *distBoxesPP;
    int *distBoxesNPP;

    std::vector<std::pair<ACTION, SOKOBAN_STATE>> getSucc(SOKOBAN_STATE &state);
    bool isGoalState(SOKOBAN_STATE &state);
    SOKOBAN_NODE* makeRootNode();
    SOKOBAN_NODE* makeNode(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state);
    SOKOBAN_NODE* makeNodePreSearch(SOKOBAN_NODE* prt, ACTION action, SOKOBAN_STATE &state, POSITION goal, int gValue);
    SOKOBAN_SOLUTION extractPath(SOKOBAN_NODE* n, std::vector<SOKOBAN_NODE*> &nodes, long long int expanded, long long int addedToOpen, long long int maximumOpenSize);

    //Aux Functions
    bool checkFrozen(SOKOBAN_STATE &state, POSITION boxPosition);
    bool isFrozen(SOKOBAN_STATE &state, POSITION boxPosition, bool xFrozen, bool yFrozen, bool *treatWall);
    bool movedBox(SOKOBAN_STATE &state, ACTION action);
    ACTION opposite(ACTION action);
    int getTileDistance(POSITION a, POSITION b);
    int getBoxTileDistance(POSITION box, POSITION goal);
    bool checkMoveCloser(SOKOBAN_STATE &state, ACTION action);
    void calcTileDistanceBoxes(SOKOBAN_STATE &state, POSITION p1, int *distances);
    void calculateTileDistances();
    void calculateBoxTileDistances();
};
