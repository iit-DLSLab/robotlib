#ifndef _ROBOTLIB_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_HPP_

namespace robotlib
{    
    class RobotBase;

    template <class Data>
    class LinkDataMap : public DataMap<Link, Data>
    {

    public:
        using DataMap<Link, Data>::operator=;
        friend class RobotBase;
        ~LinkDataMap();

    private:
        LinkDataMap(const std::shared_ptr<RobotBase>);
        LinkDataMap(const std::shared_ptr<RobotBase>, const std::shared_ptr<Data>);
    };
}

#include "link_data_map.tpp"


#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_