/*!
 * @file utils.hpp
 *
 * @brief File storing utility classes and structs.
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

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
    struct Iterator
    {
    public:
        //! Category of the iterator.
        using iterator_category = std::forward_iterator_tag;
        //! Difference type.
        using difference_type = std::ptrdiff_t;
        //! Data type.
        using value_type = Data;
        //! Pointer to the data type.
        using pointer = Data *;
        //! Reference to the data type.
        using reference = Data &;

        /*!
         * @brief Constructor.
         * @param[in] ptr pointer of type Data*.
         */
        Iterator(pointer ptr) : m_ptr(ptr) {}

        /*!
         * @brief Operator *.
         * @return reference to the data pointed by m_ptr.
         */
        reference operator*() const { return *m_ptr; }

        /*!
         * @brief Operator ->.
         * @return pointer pointing to the data.
         */
        pointer operator->() { return m_ptr; }

        /*!
         * @brief Pre-increment version of operator ++.
         * @return reference of the iterator whose pointer has been moved forward.
         */
        Iterator &operator++()
        {
            m_ptr++;
            return *this;
        }

        /*!
         * @brief Post-increment version of operator ++.
         * @return iterator whose value corresponds to the one of *this before the increment.
         */
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        /*!
         * @brief Pre-decrement version of operator --.
         * @return reference of the iterator whose pointer has been moved backward.
         */
        Iterator &operator--()
        {
            m_ptr--;
            return *this;
        }

        /*!
         * @brief Post-decrement version of operator --.
         * @return iterator whose value corresponds to the one of *this before the decrement.
         */
        Iterator operator--(int)
        {
            Iterator tmp = *this;
            --(*this);
            return tmp;
        }

        /*!
         * @brief Implementation of operator ==.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return true or false depending on if the two imputs are equal or not respectively.
         */
        friend bool operator==(const Iterator &a, const Iterator &b) { return a.m_ptr == b.m_ptr; };

        /*!
         * @brief Implementation of operator !=.
         * @param[in] a first iterator to compare with the second one.
         * @param[in] b second iterator to compare with the first one.
         * @return false or true depending on if the two imputs are equal or not respectively.
         */
        friend bool operator!=(const Iterator &a, const Iterator &b) { return a.m_ptr != b.m_ptr; };

    protected:
        //! Pointer pointing to the data.
        pointer m_ptr;
    };

    /*!
     * @brief ContainerBase class. This class is a base class for a wrapper around a std::array object.
     * @details
     * This class is used to access to the std::array variable defined in the Container child class, without knowing its length. In this way, the ContainerBase class can be used in the RobotBase class.
     * @tparam Data data type of the object to be stored.
     */
    template <class Data>
    class ContainerBase
    {
    public:
        /*!
         * @brief Constructor.
         */
        ContainerBase();

        /*!
         * @brief Destructor.
         */
        virtual ~ContainerBase();

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the array stored in the Container child class.
        */
        virtual Iterator<const Data> begin() const = 0;

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the array stored in the Container child class.
        */
        virtual Iterator<const Data> end() const = 0;

        /*!
        * @brief Get the size of the array stored in the Container child class.
        * @return size of the array stored in the Container child class.
        */
        virtual int size() const = 0;
    };

    /*!
     * @brief Container class. This class is a wrapper around a std::array object.
     * @tparam Data data type of the object to be stored.
     * @tparam N length of the wrapped std::array variable.
     */
    template <class Data, unsigned int N>
    class Container : public ContainerBase<Data>
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] data std::array of data to be wrapped.
         */
        Container(const std::array<Data, N> data);

        /*!
        * @brief Destructor.
        */
        virtual ~Container();

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        virtual Iterator<const Data> begin() const override;

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        virtual Iterator<const Data> end() const override;

        /*!
        * @brief Get the size of the wrapped std::array.
        * @return size of the wrapped std::array.
        */
        virtual int size() const override;

    protected:
        //! std::array wrapped by the Container class.
        const std::array<Data, N> data_;
    };
} // namespace robotlib

#include "utils.tpp"

#endif // _ROBOTLIB_UTILS_HPP_