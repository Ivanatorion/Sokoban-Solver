#ifndef NODE_MANAGER_H
#define NODE_MANAGER_H

typedef struct free_l_pair{
    int nodeVector;
    int position;
} FreePair;

class NodeManager{
  public:
    NodeManager(const int alocSize);
    ~NodeManager();

    SOKOBAN_NODE* makeRootNode();
    SOKOBAN_NODE* makeNode(SOKOBAN_NODE* parent, ACTION action, const SOKOBAN_STATE &state, short int heuristic);

    void deleteNodes(std::vector<SOKOBAN_NODE*>& nodes);

    long int size() const;
    long int deletedQuant() const;

  private:
    int m_alocSize;
    int m_nodeCounter;

    int m_freeListPTR;
    int m_freeListSize;
    int m_freeListMax;

    FreePair *m_freeList;

    int m_nodesVectorMaxSize;
    int m_nodesVectorSize;
    SOKOBAN_NODE **m_nodeVectors;

    long int m_deletedQuant;
    void markDelete(SOKOBAN_NODE* node);
};

inline long int NodeManager::size() const{
    return this->m_nodeCounter + (this->m_alocSize * (this->m_nodesVectorSize-1)) - (this->m_freeListSize - this->m_freeListPTR);
}

inline long int NodeManager::deletedQuant() const{
  return this->m_deletedQuant;
}

#endif