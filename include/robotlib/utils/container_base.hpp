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
        //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;

    public:
        /*!
         * @brief Constructor.
         */
        ContainerBase() = default;

        /*!
         * @brief Destructor.
         */
        virtual ~ContainerBase() = default;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the array stored in the Container child class.
        */
        virtual Iterator<const Data> begin() const;
        
        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the array stored in the Container child class.
        */
        virtual Iterator<const Data> end() const;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        virtual Iterator<Data> begin();

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        virtual Iterator<Data> end();

        /*!
        * @brief Get the size of the array stored in the Container child class.
        * @return size of the array stored in the Container child class.
        */
        virtual unsigned int length() const;

        /*!
        * @brief Get reference to data at idx position.
        * @param[in] idx index of the data in std::array.
        */
        virtual Data& operator[](unsigned int idx);

        // /*!
        // * @brief Get reference to data at idx position.
        // * @param[in] idx index of the data in std::array.
        // */
        // virtual Data& operator[](unsigned int idx) const;

    };
} // namespace robotlib

#include "container_base.tpp"

#endif // _CONTAINER_BASE_HPP_