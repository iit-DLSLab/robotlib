#ifndef _CONTAINER_HPP_
#define _CONTAINER_HPP_

#include "container_base.hpp"
#include <memory>
#include <Eigen/Dense>

namespace robotlib
{
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

#include "container.tpp"

#endif // _CONTAINER_HPP_