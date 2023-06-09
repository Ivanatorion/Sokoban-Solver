#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"

void BoxPositionSet::emplace(const short int p){
    *(this->mem + this->curNElems) = p;
    this->curNElems++;

    int j = this->curNElems - 1;
    POSITION aux;
    while(j > 0 && this->mem[j] < this->mem[j-1]){
        aux = this->mem[j];
        this->mem[j] = this->mem[j-1];
        this->mem[j-1] = aux;
        j--;
    }
}

void BoxPositionSet::insert(const short int p){
    *(this->mem + this->curNElems) = p;
    this->curNElems++;

    int j = this->curNElems - 1;
    POSITION aux;
    while(j > 0 && this->mem[j] < this->mem[j-1]){
        aux = this->mem[j];
        this->mem[j] = this->mem[j-1];
        this->mem[j-1] = aux;
        j--;
    }
}

SetIterator BoxPositionSet::erase( SetIterator pos ){
    SetIterator returnV = pos;
    while(pos != this->end()){
        SetIterator pPos = pos;
        pos++;
        if(pos != this->end())
            *pPos = *pos;
    }
    this->curNElems--;
    return returnV;
}
