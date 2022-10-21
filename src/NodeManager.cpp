#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"
#include "../include/Sokoban.h"
#include "../include/NodeManager.h"

#define START_NODE_VECTOR_MAX_SIZE 4

NodeManager::NodeManager(const int deleteSize, const int alocSize) : 
    m_deleteSize(deleteSize), m_alocSize(alocSize),
    m_nodeCounter(0), m_freeCounter(0),
    m_nodesVectorMaxSize(START_NODE_VECTOR_MAX_SIZE), m_nodesVectorSize(1)
{
    this->m_nodeVectors = (SOKOBAN_NODE**) malloc(sizeof(SOKOBAN_NODE*) * START_NODE_VECTOR_MAX_SIZE);
    this->m_nodeVectors[0] = (SOKOBAN_NODE*) malloc(sizeof(SOKOBAN_NODE) * this->m_alocSize);

    this->m_freeList = (FreePair *) malloc(sizeof(FreePair) * this->m_deleteSize);
}

NodeManager::~NodeManager(){
    for(int i = 0; i < this->m_nodesVectorSize-1; i++){
        for(int j = 0; j < this->m_alocSize; j++){
            SOKOBAN_NODE *node = this->m_nodeVectors[i] + j;
            if(node->state != nullptr)
                delete node->state;
        }
        free(this->m_nodeVectors[i]);
    }
    for(int j = 0; j < this->m_nodeCounter; j++){
        SOKOBAN_NODE *node = this->m_nodeVectors[this->m_nodesVectorSize-1] + j;
        if(node->state != nullptr)
            delete node->state;
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

    if(this->m_freeCounter > 0){
        this->m_freeCounter--;
        node = this->m_nodeVectors[this->m_freeList[this->m_freeCounter].nodeVector] + this->m_freeList[this->m_freeCounter].position;
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
    node->state = new SOKOBAN_STATE;
    *(node->state) = state;
    node->heuristica = heuristic;
    node->pathCost = node->parent->pathCost + 1;
    node->additionalF = 0;
    node->parent->nSuccs = node->parent->nSuccs + 1;
    node->nSuccs = 0;
    return node;
}

//Delete only states
void NodeManager::deleteNodes(std::vector<SOKOBAN_NODE*>& nodes){
    
    this->m_freeCounter = 0;
    
    const int nSize = (int) nodes.size();
    for(int i = 0; i < nSize; i++){
        SOKOBAN_NODE* node = nodes[i];
        if(node->state != nullptr){
            delete node->state;
            node->state = nullptr;
        }
        
        for(int j = 0; j < this->m_nodesVectorSize; j++){
            uintptr_t ui_node = reinterpret_cast<std::uintptr_t>(node);
            uintptr_t ui_vector = reinterpret_cast<std::uintptr_t>(this->m_nodeVectors[j]);
            uintptr_t ui_diff = (ui_node - ui_vector) / sizeof(SOKOBAN_NODE);
            if(ui_diff >= 0 && ui_diff < static_cast<uint64_t>(this->m_alocSize)){
                this->m_freeList[this->m_freeCounter].nodeVector = j;
                this->m_freeList[this->m_freeCounter].position = static_cast<int>(ui_diff);
                this->m_freeCounter++;
            }
        }
    }

    nodes.erase(nodes.begin(), nodes.begin() + nSize);
    nodes.reserve(this->m_deleteSize);
}

long int NodeManager::size() const{
    return this->m_nodeCounter + (this->m_alocSize * (this->m_nodesVectorSize-1)) - this->m_freeCounter;
}
