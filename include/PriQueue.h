#ifndef PRI_QUEUE_H
#define PRI_QUEUE_H

#define PRI_Q_ALOC_SIZE 1000000

class PriQueue{
  public:
    PriQueue(const int heapBranch);
    ~PriQueue();

    SOKOBAN_NODE* deleteMin();
    void push(SOKOBAN_NODE* node, int priority);
    int size() const;
    bool empty() const;

  private:
    int heapBranch;

    int *heap;
    SOKOBAN_NODE** heapNodes; //Which node is in position
    int heapSize, maxHeapSize;

    void swapHeap(int p1, int p2);
    void fixHeapDown(int p);
    void fixHeapUp(int p);
};

inline SOKOBAN_NODE* PriQueue::deleteMin(){
  if(this->empty())
    return NULL;

  SOKOBAN_NODE* returnValue = this->heapNodes[0];

  swapHeap(0, this->heapSize - 1);
  this->heapSize--;

  fixHeapDown(0);

  returnValue->nUseCount--;
  return returnValue;
}

inline int PriQueue::size() const{
  return this->heapSize;
}

inline bool PriQueue::empty() const{
  return this->heapSize == 0;
}

inline void PriQueue::push(SOKOBAN_NODE* node, int priority){
  if(this->heapSize == this->maxHeapSize){
    this->maxHeapSize = this->maxHeapSize + PRI_Q_ALOC_SIZE;
    int *tempHeap = (int *) malloc(this->maxHeapSize * sizeof(int *));
    SOKOBAN_NODE **tempHeapNodes = (SOKOBAN_NODE **) malloc(this->maxHeapSize * sizeof(SOKOBAN_NODE **));
  
    memcpy(tempHeap, this->heap, (this->maxHeapSize - PRI_Q_ALOC_SIZE) * sizeof(int));
    memcpy(tempHeapNodes, this->heapNodes, (this->maxHeapSize - PRI_Q_ALOC_SIZE) * sizeof(SOKOBAN_NODE *));
    free(this->heap);
    free(this->heapNodes);
    this->heap = tempHeap;
    this->heapNodes = tempHeapNodes;
  }

  this->heapNodes[this->heapSize] = node;
  this->heap[this->heapSize] = priority;
  this->heapSize++;

  fixHeapUp(this->heapSize - 1);

  node->nUseCount++;
}

inline void PriQueue::swapHeap(int p1, int p2){
  int aux = this->heap[p1];
  this->heap[p1] = this->heap[p2];
  this->heap[p2] = aux;
  SOKOBAN_NODE *aux2 = this->heapNodes[p1];
  this->heapNodes[p1] = this->heapNodes[p2];
  this->heapNodes[p2] = aux2;
}

inline void PriQueue::fixHeapDown(int p){
  if(p * this->heapBranch + 1 >= this->heapSize)
    return;

  int minChild = p * this->heapBranch + 1;

  for(int i = 2; i <= this->heapBranch; i++)
    if((p * this->heapBranch + i < this->heapSize) && (this->heap[minChild] > this->heap[p * this->heapBranch + i]))
      minChild = p * this->heapBranch + i;

  if(this->heap[p] > this->heap[minChild]){
    swapHeap(p, minChild);
    fixHeapDown(minChild);
  }
}

inline void PriQueue::fixHeapUp(int p){
  if(p == 0)
    return;

  if(this->heap[(p - 1) / this->heapBranch] > this->heap[p]){
    swapHeap(p, (p - 1) / this->heapBranch);
    fixHeapUp((p - 1) / this->heapBranch);
  }
}

#endif
