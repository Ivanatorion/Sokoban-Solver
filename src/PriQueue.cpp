#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/PriQueue.h"

PriQueue::PriQueue(const int heapBranch){
  this->heapBranch = heapBranch;
  
  this->heap = (int *) malloc(PRI_Q_ALOC_SIZE * sizeof(int *));
  this->heapNodes = (SOKOBAN_NODE **) malloc(PRI_Q_ALOC_SIZE * sizeof(SOKOBAN_NODE **));
  this->maxHeapSize = PRI_Q_ALOC_SIZE;
  this->heapSize = 0;
}

PriQueue::~PriQueue(){
  free(this->heap);
  free(this->heapNodes);
}
