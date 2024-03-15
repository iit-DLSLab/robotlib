#ifndef _CONTAINER_BASE_HPP_
#define _CONTAINER_BASE_HPP_

#include "iterator_container.hpp"

#include <memory>
#include <Eigen/Dense>

namespace robotlib
{
    /*!
     * @brief ContainerAbstract class. This class is a base class for a wrapper around a std::array object.
     * @details
     * This class is used to access to the std::array variable defined in the Container child class, without knowing its length. In this way, the ContainerAbstract class can be used in the RobotBase class.
     * @tparam Data data type of the object to be stored.
     */
    template <class Data>
    class ContainerAbstract
    {
        //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;

    public:
        /*!
         * @brief Constructor.
         */
        ContainerAbstract() = default;

        /*!
         * @brief Destructor.
         */
        virtual ~ContainerAbstract() = default;

        // /*!
        // * @brief Begin function to be used with iterators.
        // * @return iterator object pointing to the first data of the array stored in the Container child class.
        // */
        // virtual IteratorContainer<Data> begin() const = 0;
        
        // /*!
        // * @brief End function to be used with iterators.
        // * @return iterator object pointing to the last data of the array stored in the Container child class.
        // */
        // virtual IteratorContainer<Data> end() const = 0;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        virtual IteratorContainer<Data> begin() = 0;

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        virtual IteratorContainer<Data> end() = 0;

        /*!
        * @brief Get the size of the array stored in the Container child class.
        * @return size of the array stored in the Container child class.
        */
        virtual unsigned int length() const = 0;

        /*!
        * @brief Get reference to data at idx position.
        * @param[in] idx index of the data in std::array.
        */
        virtual Data& operator[](unsigned int idx) = 0;


        virtual Data*& at(unsigned int idx) = 0;

    };


    template <class Data>
    class ContainerBase : public std::shared_ptr<ContainerAbstract<Data>>
    {
        //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;

    public:
        /*!
         * @brief Constructor.
         */
        ContainerBase() : std::shared_ptr<ContainerAbstract<Data>>() {};

         /*!
         * @brief Constructor.
         */
        ContainerBase(ContainerAbstract<Data>& rhd) : std::shared_ptr<ContainerAbstract<Data>>(&rhd) {};

        /*!
         * @brief Destructor.
         */
        virtual ~ContainerBase() = default;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        virtual IteratorContainer<Data> begin() { return this->begin(); }

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        virtual IteratorContainer<Data> end() { return this->end(); }


        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        virtual const IteratorContainer<Data> begin() const { return this->begin(); }

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        virtual const IteratorContainer<Data> end() const { return this->end(); }
    };
} // namespace robotlib

#endif // _CONTAINER_BASE_HPP_