#ifndef _ITERATOR_HPP_
#define _ITERATOR_HPP_

namespace robotlib
{
template <class Data>
    class Iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Data;
        using pointer = Data *;
        using reference = Data &;
        Iterator(pointer ptr);
        reference operator*() const;
        pointer operator->();
        Iterator &operator++();
        Iterator operator++(int);

        Iterator &operator--();

        Iterator operator--(int);

        friend bool operator==(const Iterator &a, const Iterator &b);
        friend bool operator!=(const Iterator &a, const Iterator &b);

    protected:
        pointer m_ptr;
    };
}

#include "utils/iterator.tpp"

#endif // _ITERATOR_HPP_