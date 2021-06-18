#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

namespace dls
{
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
            friend bool operator==(const Iterator &a, const Iterator &b) { return a.m_ptr == b.m_ptr; };
            friend bool operator!=(const Iterator &a, const Iterator &b) { return a.m_ptr != b.m_ptr; };

        protected:
            pointer m_ptr;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_UTILS_HPP_
