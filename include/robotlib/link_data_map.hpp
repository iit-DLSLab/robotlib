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
        LinkDataMap(RobotBase *robot);
        LinkDataMap(RobotBase *robot, const Data &data);
    };
}

#include "link_data_map.tpp"


#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_