#ifndef STATE_SET_H
#define STATE_SET_H

//#define STATE_SET_TABLE_SIZE 10402559
#define STATE_SET_TABLE_SIZE 3002773
//#define STATE_SET_TABLE_SIZE 1001933
//#define STATE_SET_TABLE_SIZE 302317
//#define STATE_SET_TABLE_SIZE 201919
//#define STATE_SET_TABLE_SIZE 9767

#pragma pack(push, 1)
class StateSet {
    public:
        StateSet(const int boxesPerState);
        ~StateSet();

        bool isInSet(const struct SOKOBAN_STATE& state) const;
        void insert(struct SOKOBAN_STATE& state);
        void erase(const struct SOKOBAN_STATE& state);

        void shrink(const char gameMap[], const POSITION mapLenX, const POSITION mapLenY, const bool verbose);
        
        long long int size() const;

    private:
        long long int m_size;
        int m_boxesPerState;
        POSITION **m_tableBPSet;
        POSITION **m_tablePPosition;

        int *m_tableBucketSizes;
        int *m_tableBucketSizesMax;

        size_t hashState(const struct SOKOBAN_STATE& state) const;
};
#pragma pack(pop)

inline long long int StateSet::size() const{
    return this->m_size;
}

inline size_t StateSet::hashState(const struct SOKOBAN_STATE& state) const{ 
    size_t result = state.playerPosition;
    int i = 0;
    for(POSITION x : state.boxPositions){
        result = result ^ ((size_t) (static_cast<POSITION>(x * 317)) << ((i%4 + 1)*16));
        i++;
    }
    return (result << 37) | (result >> (64 - 37));
}

inline bool StateSet::isInSet(const struct SOKOBAN_STATE& state) const{
    const size_t HASH = this->hashState(state) % STATE_SET_TABLE_SIZE;

    int i = this->m_tableBucketSizes[HASH] - 1;
    while(i >= 0){
        if(state.playerPosition == this->m_tablePPosition[HASH][i] && state.boxPositions == &this->m_tableBPSet[HASH][i * m_boxesPerState])
            return true;
        i--;
    }

    return false;
}

inline void StateSet::insert(struct SOKOBAN_STATE& state){
    this->m_size++;
    const size_t HASH = this->hashState(state) % STATE_SET_TABLE_SIZE;

    const int bSize = this->m_tableBucketSizes[HASH];
    
    if(bSize == this->m_tableBucketSizesMax[HASH]){
        this->m_tableBucketSizesMax[HASH] = this->m_tableBucketSizesMax[HASH] * 2;
        POSITION *newBucketPP = new POSITION[this->m_tableBucketSizesMax[HASH]];
        POSITION *newBucketBPSet = new POSITION[this->m_tableBucketSizesMax[HASH] * this->m_boxesPerState];
        memcpy(newBucketPP, this->m_tablePPosition[HASH], (this->m_tableBucketSizesMax[HASH] / 2) * sizeof(POSITION));
        memcpy(newBucketBPSet, this->m_tableBPSet[HASH], (this->m_tableBucketSizesMax[HASH] / 2) * this->m_boxesPerState * sizeof(POSITION));
        delete[] this->m_tablePPosition[HASH];
        delete[] this->m_tableBPSet[HASH];
        this->m_tableBPSet[HASH] = newBucketBPSet;
        this->m_tablePPosition[HASH] = newBucketPP;
    }

    this->m_tablePPosition[HASH][bSize] = state.playerPosition;
    memcpy(&this->m_tableBPSet[HASH][bSize * m_boxesPerState], state.boxPositions.getMemPTR(), this->m_boxesPerState * sizeof(POSITION));
    this->m_tableBucketSizes[HASH]++;
}

inline void StateSet::erase(const struct SOKOBAN_STATE& state){
    const size_t HASH = this->hashState(state) % STATE_SET_TABLE_SIZE;

    const int bSize = this->m_tableBucketSizes[HASH];

    int i = 0;
    while(i < bSize){
        if(state.playerPosition == this->m_tablePPosition[HASH][i] && state.boxPositions == &this->m_tableBPSet[HASH][i * m_boxesPerState]){
            for(int j = i + 1; j < bSize; j++){
                m_tablePPosition[HASH][j - 1] = m_tablePPosition[HASH][j];
                memcpy(&m_tableBPSet[HASH][(j - 1) * m_boxesPerState], &m_tableBPSet[HASH][(j) * m_boxesPerState], m_boxesPerState * sizeof(POSITION));
            }
            this->m_tableBucketSizes[HASH]--;
            this->m_size--;
            return;
        }
        i++;
    }
}

#endif