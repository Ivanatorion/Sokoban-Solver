#include "../include/pcheaders.h"
#include "../include/BoxPositionSet.h"

SetIterator::SetIterator(POSITION* ptr) : m_ptr(ptr) {}

POSITION& SetIterator::operator*() const { return *m_ptr; }
POSITION* SetIterator::operator->() { return m_ptr; }

SetIterator& SetIterator::operator++() { m_ptr++; return *this; }  
SetIterator SetIterator::operator++(int) { SetIterator tmp = *this; ++(*this); return tmp; }

bool operator== (const SetIterator& a, const SetIterator& b) { return a.m_ptr == b.m_ptr; };
bool operator!= (const SetIterator& a, const SetIterator& b) { return a.m_ptr != b.m_ptr; }; 

ConstantSetIterator::ConstantSetIterator(POSITION* ptr) : m_ptr(ptr) {}

const POSITION& ConstantSetIterator::operator*() const { return *m_ptr; }
const POSITION* ConstantSetIterator::operator->() { return m_ptr; }

ConstantSetIterator& ConstantSetIterator::operator++() { m_ptr++; return *this; }
ConstantSetIterator ConstantSetIterator::operator++(int) { ConstantSetIterator tmp = *this; ++(*this); return tmp; }

bool operator== (const ConstantSetIterator& a, const ConstantSetIterator& b) { return a.m_ptr == b.m_ptr; };
bool operator!= (const ConstantSetIterator& a, const ConstantSetIterator& b) { return a.m_ptr != b.m_ptr; }; 

BoxPositionSet::BoxPositionSet() : curNElems(0){
    this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
}

BoxPositionSet::BoxPositionSet(const BoxPositionSet& ls1) : curNElems(ls1.curNElems){
    this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
    memcpy(this->mem, ls1.mem, sizeof(POSITION) * this->curNElems);
}

BoxPositionSet::BoxPositionSet(BoxPositionSet&& source) : mem(source.mem), curNElems(source.curNElems){
    source.mem = NULL;
}

BoxPositionSet& BoxPositionSet::operator=(const BoxPositionSet& source){
    this->curNElems = source.curNElems;
    //free(this->mem);
    //this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
    memcpy(this->mem, source.mem, sizeof(POSITION) * this->curNElems);
    return *this;
}

BoxPositionSet::~BoxPositionSet(){
    free(this->mem);
}

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

SetIterator BoxPositionSet::find(const short int p){
    SetIterator itr = this->begin();
    while(itr != this->end() && *itr != p)
        itr++;
    return itr;
}

ConstantSetIterator BoxPositionSet::find(const short int p) const{
    ConstantSetIterator itr = this->begin();
    while(*itr != p && itr != this->end())
        itr++;
    return itr;
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

SetIterator BoxPositionSet::begin() {return SetIterator(this->mem);}
SetIterator BoxPositionSet::end() {return SetIterator(this->mem + this->curNElems);}
ConstantSetIterator BoxPositionSet::begin() const {return ConstantSetIterator(this->mem);}
ConstantSetIterator BoxPositionSet::end() const {return ConstantSetIterator(this->mem + this->curNElems);}