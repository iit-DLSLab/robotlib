#ifndef _CONTAINER_BASE_HPP_
#define _CONTAINER_BASE_HPP_

#include "iterator.hpp"

#include <memory>
#include <Eigen/Dense>

namespace robotlib
{
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

        //TODO: define [] operator using a string as input (it implies that all the possible Data classes have a getName function)

        /*!
        * @brief Get the size of the array stored in the Container child class.
        * @return size of the array stored in the Container child class.
        */
        virtual int size() const = 0;
    };
} // namespace robotlib

#include "container_base.tpp"

#endif // _CONTAINER_BASE_HPP_