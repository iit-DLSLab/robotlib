#ifndef _ITERATOR_DATA_MAP_HPP_
#define _ITERATOR_DATA_MAP_HPP_
#include <memory>
#include <iostream>
#include "../data_pair.hpp"

namespace robotlib
{
    /*!
     * @brief Iterator struct.
     * @details
     * This struct is used to iterate over custom data types.
     * @tparam Data data type of the object on which iterate on.
     */
    template <class Key, class Data>
    struct IteratorDataMap
    {

    public:
        /*!
         * @brief Constructor.
         * @param[in] ptr pointer of type Data*.
         */
        IteratorDataMap(DataPair<Key, Data>* ptr, unsigned int count = 0) 
            : m_ptr(ptr)
            , counter{count}
        {}

        /*!
         * @brief Constructor.
         * @param[in] ptr pointer of type Data*.
         */
        IteratorDataMap(std::vector<Data*>& vec, unsigned int count = 0) 
            : m_ptr(vec.data())
            , counter{count}
        {}

        /*!
         * @brief Operator *.
         * @return reference to the data pointed by m_ptr.
         */
        DataPair<Key, Data>& operator*() { return m_ptr[counter]; }

        /*!
         * @brief Operator *.
         * @return reference to the data pointed by m_ptr.
         */
        const DataPair<Key, Data>& operator*() const { return &m_ptr[counter]; }

        /*!
         * @brief Operator ->.
         * @return pointer pointing to the data.
         */
        DataPair<Key, Data>* operator->() { return &m_ptr[counter]; }

        /*!
         * @brief Operator ->.
         * @return pointer pointing to the data.
         */
        DataPair<Key, Data>* get() { return m_ptr[counter]; }

        /*!
         * @brief Pre-increment version of operator ++.
         * @return reference of the iterator whose pointer has been moved forward.
         */
        IteratorDataMap& operator++()
        {
            counter++;
            return *this;
        }

        /*!
         * @brief Post-increment version of operator ++.
         * @return iterator whose value corresponds to the one of *this before the increment.
         */
        IteratorDataMap operator++(int)
        {
            IteratorDataMap tmp = *this;
            counter++;
            return tmp;
        }

        /*!
         * @brief Pre-decrement version of operator --.
         * @return reference of the iterator whose pointer has been moved backward.
         */
        IteratorDataMap& operator--()
        {
            counter--;
            return *this;
        }

        /*!
         * @brief Post-decrement version of operator --.
         * @return iterator whose value corresponds to the one of *this before the decrement.
         */
        IteratorDataMap operator--(int)
        {
            IteratorDataMap tmp = *this;
            counter--;
            return tmp;
        }

        /*!
         * @brief Implementation of operator ==.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return true or false depending on if the two imputs are equal or not respectively.
         */
        friend bool operator==(const IteratorDataMap& a, const IteratorDataMap& b) { return (a.counter == b.counter); };

        /*!
         * @brief Implementation of operator !=.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return false or true depending on if the two imputs are equal or not respectively.
         */
        friend bool operator!=(const IteratorDataMap& a, const IteratorDataMap& b) { return (a.counter != b.counter); };

    protected:
        //! Pointer pointing to the data.
        DataPair<Key, Data>* m_ptr;

        unsigned int counter;
    };
} // namespace robotlib
#endif // _ITERATOR_DATA_MAP_HPP_