#ifndef NODE_MANAGER_H
#define NODE_MANAGER_H

typedef struct free_l_pair{
    int nodeVector;
    int position;
} FreePair;

class NodeManager{
  public:
    NodeManager(const int deleteSize, const int alocSize);
    ~NodeManager();

    SOKOBAN_NODE* makeRootNode();
    SOKOBAN_NODE* makeNode(SOKOBAN_NODE* parent, ACTION action, const SOKOBAN_STATE &state, short int heuristic);

    void deleteNodes(std::vector<SOKOBAN_NODE*>& nodes);

    long int size() const;

  private:
    int m_deleteSize, m_alocSize;
    int m_nodeCounter;
    int m_freeCounter;

    FreePair *m_freeList;

    int m_nodesVectorMaxSize;
    int m_nodesVectorSize;
    SOKOBAN_NODE **m_nodeVectors;
};

#endif