#ifndef _ITERATOR_CONTAINER_HPP_
#define _ITERATOR_CONTAINER_HPP_
#include <memory>

namespace robotlib
{
    /*!
     * @brief Iterator struct.
     * @details
     * This struct is used to iterate over custom data types.
     * @tparam Data data type of the object on which iterate on.
     */
    template <class Data>
    struct IteratorContainer
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] ptr pointer of type Data*.
         */
        IteratorContainer(const std::shared_ptr<Data>* ptr) : m_ptr(ptr) {}

        /*!
         * @brief Operator *.
         * @return reference to the data pointed by m_ptr.
         */
        Data& operator*() { return **m_ptr; }

        /*!
         * @brief Operator *.
         * @return reference to the data pointed by m_ptr.
         */
        const Data& operator*() const { return **m_ptr; }

        /*!
         * @brief Operator ->.
         * @return pointer pointing to the data.
         */
        std::shared_ptr<Data> operator->() { return m_ptr; }

        /*!
         * @brief Operator ->.
         * @return pointer pointing to the data.
         */
        std::shared_ptr<Data>* get() { return *m_ptr; }

        /*!
         * @brief Pre-increment version of operator ++.
         * @return reference of the iterator whose pointer has been moved forward.
         */
        IteratorContainer& operator++()
        {
            m_ptr++;
            return *this;
        }

        /*!
         * @brief Post-increment version of operator ++.
         * @return iterator whose value corresponds to the one of *this before the increment.
         */
        IteratorContainer operator++(int)
        {
            IteratorContainer tmp = *this;
            ++(*this);
            return tmp;
        }

        /*!
         * @brief Pre-decrement version of operator --.
         * @return reference of the iterator whose pointer has been moved backward.
         */
        IteratorContainer& operator--()
        {
            m_ptr--;
            return *this;
        }

        /*!
         * @brief Post-decrement version of operator --.
         * @return iterator whose value corresponds to the one of *this before the decrement.
         */
        IteratorContainer operator--(int)
        {
            IteratorContainer tmp = *this;
            --(*this);
            return tmp;
        }

        /*!
         * @brief Implementation of operator ==.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return true or false depending on if the two imputs are equal or not respectively.
         */
        friend bool operator==(const IteratorContainer& a, const IteratorContainer& b) { return *(a.m_ptr) == *(b.m_ptr); };

        /*!
         * @brief Implementation of operator !=.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return false or true depending on if the two imputs are equal or not respectively.
         */
        friend bool operator!=(const IteratorContainer& a, const IteratorContainer& b) { return *(a.m_ptr) != *(b.m_ptr); };

    protected:
        //! Pointer pointing to the data.
        const std::shared_ptr<Data>* m_ptr;
    };
} // namespace robotlib
#endif // _ITERATOR_CONTAINER_HPP_