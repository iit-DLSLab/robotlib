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
     * @tparam Length length of the wrapped std::array variable.
     */
    template <class Data, unsigned int Length>
    class Container : public ContainerAbstract<Data>
    {
        //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;

    public:

        /*!
         * @brief Default constructor.
         */
        Container();

        /*!
         * @brief Constructor.
         * @param[in] data std::array of data to be wrapped.
         */
        Container(std::array<std::shared_ptr<Data>, Length> data);

        // /*!
        //  * @brief Constructor.
        //  * @param[in] data std::array of data to be wrapped.
        //  */
        // Container(std::array<Data, Length>& data);

        /*!
        * @brief Destructor.
        */
        virtual ~Container() = default;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        IteratorContainer<Data> begin() const override;

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        IteratorContainer<Data> end() const override;

        /*!
        * @brief Begin function to be used with iterators.
        * @return iterator object pointing to the first data of the data_ variable.
        */
        IteratorContainer<Data> begin() override;

        /*!
        * @brief End function to be used with iterators.
        * @return iterator object pointing to the last data of the data_ variable.
        */
        IteratorContainer <Data> end() override;

        /*!
        * @brief Get the size of the wrapped std::array.
        * @return size of the wrapped std::array.
        */
        unsigned int length() const override;

        /*!
        * @brief Get reference to data at idx position.
        * @param[in] idx index of the data in std::array.
        */
        Data& operator[](unsigned int idx) override;


        std::shared_ptr<Data>& at(unsigned int idx) override;

    private:
    
        //! std::array wrapped by the Container class.
        std::array<std::shared_ptr<Data>, Length> data_;
    };

} // namespace robotlib

#include "container.tpp"

#endif // _CONTAINER_HPP_