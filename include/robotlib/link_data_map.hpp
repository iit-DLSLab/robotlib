#ifndef _ROBOTLIB_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_HPP_

#include "link.hpp"
namespace robotlib
{    
    /*!
     * @brief LinkDataMap class.
     * @details
     * This templated class is used to store data for each link.
     * @tparam Data class of the data associated to link.
    */
    template <class Data>
    class LinkDataMap : public DataMap<Link, Data>
    {
        //! RobotBase is a friend class to let it use the private costructor of the LinkDataMap class.
        friend class RobotBase;

    public:
        LinkDataMap(const LinkDataMap&);

        /*!
         * @brief Destructor.
         */
        virtual ~LinkDataMap();

        using DataMap<Link, Data>::operator=;
        LinkDataMap<Data> &operator=(const LinkDataMap<Data> &);

    private:
        /*!
         * @brief Constructor.
         * @param[in] links list of links to map the data.
         * @param[in] data data to be associated to all links.
         */
        LinkDataMap(const ContainerBase<Link>& links, const Data& data);

        /*!
         * @brief Constructor.
         * @param[in] links list of links pointers to map the data.
         * @param[in] data data to be associated to all links.
         */
        LinkDataMap(const ContainerBase<std::shared_ptr<Link>>& links, const Data& data);

        /*!
         * @brief Constructor.
         * @param[in] links list of links to be map the data.
         * @param[in] data data list to be associated to each link.
         */
        LinkDataMap(const ContainerBase<Link>& links, const std::vector<Data>& data);
    };
}

#include "link_data_map.tpp"

#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_