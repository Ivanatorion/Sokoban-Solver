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

        ~BoxPositionSet();

        void emplace(const short int p);

        void insert(const short int p);

        SetIterator find(const short int p);

        ConstantSetIterator find(const short int p) const;

        SetIterator erase( SetIterator pos );

        SetIterator begin();
        SetIterator end();

        ConstantSetIterator begin() const;
        ConstantSetIterator end() const;

    private:
        POSITION *mem;
        short int curNElems;
};
#pragma pack(pop)

#endif