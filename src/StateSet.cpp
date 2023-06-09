#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/StateSet.h"
#include "../include/ScopeTimer.h"

#define INITIAL_BUCKET_SIZE 4

StateSet::StateSet(const int boxesPerState) : m_size(0), m_boxesPerState(boxesPerState) {
    this->m_tableBPSet = (POSITION **) malloc(STATE_SET_TABLE_SIZE * sizeof(POSITION *));
    this->m_tablePPosition = (POSITION **) malloc(STATE_SET_TABLE_SIZE * sizeof(POSITION *));
    this->m_tableBucketSizes = (int *) malloc(STATE_SET_TABLE_SIZE * sizeof(int));
    this->m_tableBucketSizesMax = (int *) malloc(STATE_SET_TABLE_SIZE * sizeof(int));

    for(int i = 0; i < STATE_SET_TABLE_SIZE; i++){
        this->m_tableBPSet[i] = new POSITION[INITIAL_BUCKET_SIZE * m_boxesPerState];
        this->m_tablePPosition[i] = new POSITION[INITIAL_BUCKET_SIZE];
        this->m_tableBucketSizes[i] = 0;
        this->m_tableBucketSizesMax[i] = INITIAL_BUCKET_SIZE;
    }
}

StateSet::~StateSet() {
    for(int i = 0; i < STATE_SET_TABLE_SIZE; i++){
        delete[] this->m_tableBPSet[i];
        delete[] this->m_tablePPosition[i];
    }
    free(this->m_tableBPSet);
    free(this->m_tablePPosition);
    free(this->m_tableBucketSizes);
    free(this->m_tableBucketSizesMax);
}

std::vector<SOKOBAN_STATE> getPred(const SOKOBAN_STATE &state, const char gameMap[], const POSITION mapLenX, const POSITION mapLenY){
  std::vector<SOKOBAN_STATE> preds;
  SOKOBAN_STATE pred;
  POSITION p;

  p = state.playerPosition - mapLenX;
  if(p >= 0 && gameMap[p] != WALL && state.boxPositions.find(p) == state.boxPositions.end()){ 
    pred.boxPositions = state.boxPositions;
    pred.playerPosition = p;
    preds.push_back(pred);
   
    p = state.playerPosition + mapLenX;
    if(state.boxPositions.find(p) != state.boxPositions.end()){
        pred.boxPositions.erase(pred.boxPositions.find(p));
        pred.boxPositions.insert(state.playerPosition);
        preds.push_back(pred);
    }	
  }

  p = state.playerPosition + mapLenX;
  if(p < mapLenX * mapLenY && gameMap[p] != WALL && state.boxPositions.find(p) == state.boxPositions.end()){
    pred.boxPositions = state.boxPositions;
    pred.playerPosition = p;
    preds.push_back(pred);
   
    p = state.playerPosition - mapLenX;
    if(state.boxPositions.find(p) != state.boxPositions.end()){
        pred.boxPositions.erase(pred.boxPositions.find(p));
        pred.boxPositions.insert(state.playerPosition);
        preds.push_back(pred);
    }	
  }

  if(state.playerPosition % mapLenX != 0){
    p = state.playerPosition - 1;
    if(gameMap[p] != WALL){
        pred.boxPositions = state.boxPositions;
        pred.playerPosition = p;
        preds.push_back(pred);
    
        p = state.playerPosition + 1;
        if(state.boxPositions.find(p) != state.boxPositions.end()){
            pred.boxPositions.erase(pred.boxPositions.find(p));
            pred.boxPositions.insert(state.playerPosition);
            preds.push_back(pred);
        }	
    }
  }

  if((state.playerPosition+1) % mapLenX != 0){
    p = state.playerPosition + 1;
    if(gameMap[p] != WALL){
        pred.boxPositions = state.boxPositions;
        pred.playerPosition = p;
        preds.push_back(pred);
    
        p = state.playerPosition - 1;
        if(state.boxPositions.find(p) != state.boxPositions.end()){
            pred.boxPositions.erase(pred.boxPositions.find(p));
            pred.boxPositions.insert(state.playerPosition);
            preds.push_back(pred);
        }	
    }
  }

  return preds;
}

void StateSet::shrink(const char gameMap[], const POSITION mapLenX, const POSITION mapLenY, const bool verbose){
    ScopeTimer timer(std::string("StateSet shrink"), verbose);

    const int before = (int) this->size();

    if(verbose)
        printf("Shrinking Closed. Before: %d\n", before);
    
    const int STATE_ALOC_SIZE = 1048576;
    const int TotalStateSize = (m_boxesPerState + 1);

    int stateListsMaxSize = 32;
    int stateListsSize = 1;
    int curStateListSize = 0;

    POSITION** stateListsBP = new POSITION*[stateListsMaxSize];
    stateListsBP[0] = new POSITION[STATE_ALOC_SIZE * TotalStateSize];

    for(int i = 0; i < STATE_SET_TABLE_SIZE; i++){
        for(int j = 0; j < this->m_tableBucketSizes[i]; j++){
            SOKOBAN_STATE s;
            s.playerPosition = this->m_tablePPosition[i][j];
            for(int k = 0; k < this->m_boxesPerState; k++)
                s.boxPositions.insert(this->m_tableBPSet[i][j * this->m_boxesPerState + k]);

            std::vector<SOKOBAN_STATE> preds = getPred(s, gameMap, mapLenX, mapLenY);
            bool allClosed = true;
            for(const SOKOBAN_STATE& p : preds){
                if(!this->isInSet(p))
                    allClosed = false;
            }
            if(allClosed){
                if(curStateListSize == STATE_ALOC_SIZE){
                    if(stateListsSize == stateListsMaxSize){
                        stateListsMaxSize = stateListsMaxSize * 2;
                        POSITION** newStateLists = new POSITION*[stateListsMaxSize];
                        memcpy(newStateLists, stateListsBP, sizeof(POSITION*) * stateListsMaxSize / 2);
                        delete[] stateListsBP;
                        stateListsBP = newStateLists;
                    }
                    stateListsBP[stateListsSize] = new POSITION[STATE_ALOC_SIZE * TotalStateSize];
                    stateListsSize++;
                    curStateListSize = 0;
                }
                stateListsBP[stateListsSize - 1][curStateListSize * TotalStateSize] = s.playerPosition;
                memcpy(&stateListsBP[stateListsSize - 1][curStateListSize * TotalStateSize + 1], s.boxPositions.getMemPTR(), sizeof(POSITION) * m_boxesPerState);
                curStateListSize++;
            }
        }
    }

    for(int i = 0; i < stateListsSize - 1; i++){
        for(int j = 0; j < STATE_ALOC_SIZE; j++){
            SOKOBAN_STATE s;
            s.playerPosition = stateListsBP[i][j * TotalStateSize];
            for(int k = 0; k < this->m_boxesPerState; k++)
                s.boxPositions.insert(stateListsBP[i][j * TotalStateSize + 1 + k]);
            this->erase(s);
        }
        delete[] stateListsBP[i];
    }

    for(int j = 0; j < curStateListSize; j++){
        SOKOBAN_STATE s;
        s.playerPosition = stateListsBP[stateListsSize - 1][j * TotalStateSize];
        for(int k = 0; k < this->m_boxesPerState; k++)
            s.boxPositions.insert(stateListsBP[stateListsSize - 1][j * TotalStateSize + 1 + k]);
        this->erase(s);
    }

    delete[] stateListsBP[stateListsSize - 1];
    delete[] stateListsBP;

    const int after = (int) this->size();

    if(verbose)
        printf("Shrinking Closed. After: %d (%.2f%% nodes deleted)\n", after, 100.0 - (100.0 * after) / before);
}