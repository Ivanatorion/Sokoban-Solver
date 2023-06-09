#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/NodeManager.h"

#define START_NODE_VECTOR_MAX_SIZE 4

NodeManager::NodeManager(const int alocSize) : 
    m_alocSize(alocSize),
    m_nodeCounter(0), m_freeListPTR(0), m_freeListSize(0),
    m_nodesVectorMaxSize(START_NODE_VECTOR_MAX_SIZE), m_nodesVectorSize(1), m_deletedQuant(0)
{
    this->m_nodeVectors = (SOKOBAN_NODE**) malloc(sizeof(SOKOBAN_NODE*) * START_NODE_VECTOR_MAX_SIZE);
    this->m_nodeVectors[0] = (SOKOBAN_NODE*) malloc(sizeof(SOKOBAN_NODE) * this->m_alocSize);

    this->m_freeList = (FreePair *) malloc(sizeof(FreePair) * this->m_alocSize);
    this->m_freeListMax = this->m_alocSize;
}

NodeManager::~NodeManager(){
    for(int i = 0; i < this->m_nodesVectorSize-1; i++){
        for(int j = 0; j < this->m_alocSize; j++){
            SOKOBAN_NODE *node = this->m_nodeVectors[i] + j;
            if(node->info != nullptr)
                delete node->info;
        }
        free(this->m_nodeVectors[i]);
    }
    for(int j = 0; j < this->m_nodeCounter; j++){
        SOKOBAN_NODE *node = this->m_nodeVectors[this->m_nodesVectorSize-1] + j;
        if(node->info != nullptr)
            delete node->info;
    }

    free(this->m_nodeVectors[this->m_nodesVectorSize-1]);
    
    free(this->m_nodeVectors);

    free(this->m_freeList);
}

SOKOBAN_NODE* NodeManager::makeRootNode(){
    return NULL;
}

SOKOBAN_NODE* NodeManager::makeNode(SOKOBAN_NODE* parent, ACTION action, const SOKOBAN_STATE &state, short int heuristic){
    SOKOBAN_NODE* node;

    if(this->m_freeListPTR < this->m_freeListSize){
        node = this->m_nodeVectors[this->m_freeList[this->m_freeListPTR].nodeVector] + this->m_freeList[this->m_freeListPTR].position;
        this->m_freeListPTR++;
        if(this->m_freeListPTR == this->m_freeListSize){
            this->m_freeListSize = 0;
            this->m_freeListPTR = 0;
        }
    }
    else{
        node = this->m_nodeVectors[this->m_nodesVectorSize-1] + this->m_nodeCounter;
        this->m_nodeCounter++;
        if(this->m_nodeCounter % this->m_alocSize == 0){
            this->m_nodeCounter = 0;
            this->m_nodeVectors[this->m_nodesVectorSize] = (SOKOBAN_NODE*) malloc(sizeof(SOKOBAN_NODE) * this->m_alocSize);
            this->m_nodesVectorSize++;
            if(this->m_nodesVectorSize == this->m_nodesVectorMaxSize){
                this->m_nodesVectorMaxSize = this->m_nodesVectorMaxSize * 2;

                SOKOBAN_NODE **oldP = this->m_nodeVectors;
                this->m_nodeVectors = (SOKOBAN_NODE**) malloc(sizeof(SOKOBAN_NODE*) * this->m_nodesVectorMaxSize);
                memcpy(this->m_nodeVectors, oldP, sizeof(SOKOBAN_NODE*) * this->m_nodesVectorMaxSize / 2);
                free(oldP);
            }
            
        }
    }

    node->parent = parent;
    node->action = action;
    node->info = new NodeInfo;
    node->info->state = state;
    node->info->heuristic = heuristic;
    node->info->pathCost = node->parent->info->pathCost + 1;
    node->info->additionalF = 0;
    node->parent->nUseCount = node->parent->nUseCount + 1;
    node->nUseCount = 0;
    return node;
}

void NodeManager::deleteNodes(std::vector<SOKOBAN_NODE*>& nodes){
    const int nSize = (int) nodes.size();
    for(int i = 0; i < nSize; i++){
        this->markDelete(nodes[i]);
    }
}

void NodeManager::markDelete(SOKOBAN_NODE* node){
    this->m_deletedQuant++;

    if(node->info != nullptr){
        delete node->info;
        node->info = nullptr;
    }

    node->parent->nUseCount = node->parent->nUseCount - 1;
    if(node->parent->nUseCount == 0)
        markDelete(node->parent);
    
    for(int j = 0; j < this->m_nodesVectorSize; j++){
        uintptr_t ui_node = reinterpret_cast<std::uintptr_t>(node);
        uintptr_t ui_vector = reinterpret_cast<std::uintptr_t>(this->m_nodeVectors[j]);
        uintptr_t ui_diff = (ui_node - ui_vector) / sizeof(SOKOBAN_NODE);
        if(ui_diff >= 0 && ui_diff < static_cast<uint64_t>(this->m_alocSize)){
            if(this->m_freeListSize == this->m_freeListMax){
                this->m_freeListMax = this->m_freeListMax * 2;
                FreePair *newFreeList = (FreePair *) malloc(sizeof(FreePair) * this->m_freeListMax);
                memcpy(newFreeList, this->m_freeList, (this->m_freeListMax / 2) * sizeof(FreePair));
                free(this->m_freeList);
                this->m_freeList = newFreeList;
            }

            this->m_freeList[this->m_freeListSize].nodeVector = j;
            this->m_freeList[this->m_freeListSize].position = static_cast<int>(ui_diff);
            this->m_freeListSize++;
        }
    }
}