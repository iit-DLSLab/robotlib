#ifndef _ROBOTLIB_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_HPP_

namespace robotlib
{    
    class RobotBase;

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
         * @param[in] robot robot object to be used to get the number of links.
         */
        LinkDataMap(const RobotBase* robot_base);

        /*!
         * @brief Constructor.
         * @param[in] robot robot object to be used to get the number of links.
         * @param[in] data data to be associated to each link.
         */
        LinkDataMap(const RobotBase* robot_base, const Data& data);
    };
}

#include "link_data_map.tpp"

#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_