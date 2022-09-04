#ifndef _ROBOTLIB_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_HPP_

namespace robotlib
{    
    class RobotBase;

    template <class Data>
    class LinkDataMap : public DataMap<Link, Data>
    {
        friend class RobotBase;

    public:
        LinkDataMap(const LinkDataMap&);
        ~LinkDataMap();

        LinkDataMap<Data> &operator=(const LinkDataMap<Data> &);
        using DataMap<Link, Data>::operator=;

    private:
        LinkDataMap(const RobotBase*);
        LinkDataMap(const RobotBase*, const Data&);
        
    };
}

#include "link_data_map.tpp"


#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_