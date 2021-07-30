#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

#include <memory>
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
            ContainerBase(){};
            virtual ~ContainerBase(){};

            virtual Iterator<const std::shared_ptr<Data>> begin() const = 0;
            virtual Iterator<const std::shared_ptr<Data>> end() const = 0;
            virtual const std::shared_ptr<Data> operator[](const int id) const = 0;
            virtual const int size() const = 0;
        };

        template <class Data, unsigned int N>
        class Container : public ContainerBase<Data>
        {
        public:
            Container(const std::array<std::shared_ptr<Data>, N> data) : data_(data){};

            virtual ~Container(){};

            virtual Iterator<const std::shared_ptr<Data>> begin() const { return Iterator<const std::shared_ptr<Data>>(&data_[0]); };
            virtual Iterator<const std::shared_ptr<Data>> end() const { return Iterator<const std::shared_ptr<Data>>(&data_[N]); };

            virtual const std::shared_ptr<Data> operator[](const int id) const { return data_[id]; };

            virtual const int size() const { return data_.size(); };

        protected:
            const std::array<std::shared_ptr<Data>, N> data_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_UTILS_HPP_
