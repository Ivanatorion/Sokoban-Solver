#ifndef BOX_POSITION_SET_H
#define BOX_POSITION_SET_H

typedef short int POSITION;

#pragma pack(push, 1)
struct SetIterator{
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = POSITION;
    using pointer           = POSITION*;
    using reference         = POSITION&;

    SetIterator(POSITION* ptr);

    POSITION& operator*() const;
    POSITION* operator->();

    // Prefix increment
    SetIterator& operator++();

    // Postfix increment
    SetIterator operator++(int);

    friend bool operator== (const SetIterator& a, const SetIterator& b);
    friend bool operator!= (const SetIterator& a, const SetIterator& b);

private:
    POSITION* m_ptr;

};

struct ConstantSetIterator{
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = POSITION;
    using pointer           = POSITION*;
    using reference         = POSITION&;

    ConstantSetIterator(POSITION* ptr);

    const POSITION& operator*() const;
    const POSITION* operator->();

    // Prefix increment
    ConstantSetIterator& operator++();

    // Postfix increment
    ConstantSetIterator operator++(int);

    friend bool operator== (const ConstantSetIterator& a, const ConstantSetIterator& b);
    friend bool operator!= (const ConstantSetIterator& a, const ConstantSetIterator& b); 

private:
    POSITION* m_ptr;

};

class BoxPositionSet{
    public:
        static int STATE_BOX_QUANT;

        BoxPositionSet();

        BoxPositionSet(const BoxPositionSet& ls1);

        BoxPositionSet(BoxPositionSet&& source);

        BoxPositionSet& operator=(const BoxPositionSet& source);
        bool operator==(const POSITION* p) const;

        ~BoxPositionSet();

        void emplace(const short int p);

        void insert(const short int p);

        void replace(const short int p1, const short int p2);

        SetIterator find(const short int p);

        ConstantSetIterator find(const short int p) const;

        SetIterator erase( SetIterator pos );

        SetIterator begin();
        SetIterator end();

        ConstantSetIterator begin() const;
        ConstantSetIterator end() const;

        POSITION *getMemPTR();

    private:
        POSITION *mem;
        short int curNElems;
};
#pragma pack(pop)

inline SetIterator::SetIterator(POSITION* ptr) : m_ptr(ptr) {}

inline POSITION& SetIterator::operator*() const { return *m_ptr; }
inline POSITION* SetIterator::operator->() { return m_ptr; }

inline SetIterator& SetIterator::operator++() { m_ptr++; return *this; }  
inline SetIterator SetIterator::operator++(int) { SetIterator tmp = *this; ++(*this); return tmp; }

inline bool operator== (const SetIterator& a, const SetIterator& b) { return a.m_ptr == b.m_ptr; };
inline bool operator!= (const SetIterator& a, const SetIterator& b) { return a.m_ptr != b.m_ptr; }; 

inline ConstantSetIterator::ConstantSetIterator(POSITION* ptr) : m_ptr(ptr) {}

inline const POSITION& ConstantSetIterator::operator*() const { return *m_ptr; }
inline const POSITION* ConstantSetIterator::operator->() { return m_ptr; }

inline ConstantSetIterator& ConstantSetIterator::operator++() { m_ptr++; return *this; }
inline ConstantSetIterator ConstantSetIterator::operator++(int) { ConstantSetIterator tmp = *this; ++(*this); return tmp; }

inline bool operator== (const ConstantSetIterator& a, const ConstantSetIterator& b) { return a.m_ptr == b.m_ptr; };
inline bool operator!= (const ConstantSetIterator& a, const ConstantSetIterator& b) { return a.m_ptr != b.m_ptr; }; 

inline BoxPositionSet::BoxPositionSet() : curNElems(0){
    this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
}

inline BoxPositionSet::BoxPositionSet(const BoxPositionSet& ls1) : curNElems(ls1.curNElems){
    this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
    memcpy(this->mem, ls1.mem, sizeof(POSITION) * this->curNElems);
}

inline BoxPositionSet::BoxPositionSet(BoxPositionSet&& source) : mem(source.mem), curNElems(source.curNElems){
    source.mem = NULL;
}

inline BoxPositionSet& BoxPositionSet::operator=(const BoxPositionSet& source){
    this->curNElems = source.curNElems;
    //free(this->mem);
    //this->mem = (POSITION*) malloc(sizeof(POSITION) * BoxPositionSet::STATE_BOX_QUANT);
    memcpy(this->mem, source.mem, sizeof(POSITION) * this->curNElems);
    return *this;
}

inline bool BoxPositionSet::operator==(const POSITION* p) const{
    int i = 0;
    while(i < this->curNElems && this->mem[i] == p[i])
        i++;

    return (i == this->curNElems);
}

inline BoxPositionSet::~BoxPositionSet(){
    free(this->mem);
}

inline SetIterator BoxPositionSet::begin() {return SetIterator(this->mem);}
inline SetIterator BoxPositionSet::end() {return SetIterator(this->mem + this->curNElems);}
inline ConstantSetIterator BoxPositionSet::begin() const {return ConstantSetIterator(this->mem);}
inline ConstantSetIterator BoxPositionSet::end() const {return ConstantSetIterator(this->mem + this->curNElems);}

inline SetIterator BoxPositionSet::find(const short int p){
    SetIterator itr = this->begin();
    while(itr != this->end() && *itr != p)
        itr++;
    return itr;
}

inline ConstantSetIterator BoxPositionSet::find(const short int p) const{
    ConstantSetIterator itr = this->begin();
    while(*itr != p && itr != this->end())
        itr++;
    return itr;
}

inline void BoxPositionSet::replace(const short int p1, const short int p2){
    short int aux;
    int i = 0;
    while(this->mem[i] != p1)
        i++;
    this->mem[i] = p2;
    if(p2 > p1){
        int j = i + 1;
        while(j < this->curNElems && this->mem[j] < this->mem[j - 1]){
            aux = this->mem[j];
            this->mem[j] = this->mem[j-1];
            this->mem[j-1] = aux;
            j++;
        }
    }
    else{
        int j = i;
        while(j > 0 && this->mem[j] < this->mem[j - 1]){
            aux = this->mem[j];
            this->mem[j] = this->mem[j-1];
            this->mem[j-1] = aux;
            j--;
        }
    }
}

inline POSITION * BoxPositionSet::getMemPTR(){
    return this->mem;
}

#endif