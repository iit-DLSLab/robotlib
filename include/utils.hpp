#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

#include <memory>

namespace robotlib
{
    /**
        * Useful functions or data structures
        */
    template <class Data>
    struct Iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Data;
        using pointer = Data *;
        using reference = Data &;
        Iterator(pointer ptr) : m_ptr(ptr) {}
        reference operator*() const { return *m_ptr; }
        pointer operator->() { return m_ptr; }
        Iterator &operator++()
        {
            m_ptr++;
            return *this;
        }
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        Iterator &operator--()
        {
            m_ptr--;
            return *this;
        }

        Iterator operator--(int)
        {
            Iterator tmp = *this;
            --(*this);
            return tmp;
        }

        friend bool operator==(const Iterator &a, const Iterator &b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!=(const Iterator &a, const Iterator &b) { return a.m_ptr != b.m_ptr; };

    protected:
        pointer m_ptr;
    };

    template <class Data>
    class ContainerBase
    {
    public:
        ContainerBase();
        virtual ~ContainerBase();

        virtual Iterator<const Data> begin() const = 0;
        virtual Iterator<const Data> end() const = 0;

        //TODO: define [] operator using a string as input (it implies that all the possible Data classes have a getName function)

        virtual const int size() const = 0;
    };

    template <class Data, unsigned int N>
    class Container : public ContainerBase<Data>
    {
    public:
        Container(const std::array<Data, N> data);

        virtual ~Container();

        virtual Iterator<const Data> begin() const override;
        virtual Iterator<const Data> end() const override;

        virtual const int size() const override;

    protected:
        const std::array<Data, N> data_;
    };
} // namespace robotlib

#include "utils.tpp"

#endif // _ROBOTLIB_UTILS_HPP_