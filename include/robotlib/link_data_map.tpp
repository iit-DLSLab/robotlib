#ifndef _ROBOTLIB_LINK_DATA_MAP_TPP_
#define _ROBOTLIB_LINK_DATA_MAP_TPP_

#include "link_data_map.hpp"

namespace robotlib
{
    template <class Data>
    LinkDataMap<Data>::~LinkDataMap(){}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const ContainerBase<Link>& links, const Data& data) 
        : DataMap<Link, Data>(links, data)
    {}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const ContainerBase<std::shared_ptr<Link>>& links, const Data& data) 
        : DataMap<Link, Data>(links, data)
    {}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const ContainerBase<Link>& links, const std::vector<Data>& data) 
        : DataMap<Link, Data>(links, data)
    {}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const LinkDataMap &other)
        : DataMap<Joint, Data>(other)
    {}

    template <class Data>
    LinkDataMap<Data> &LinkDataMap<Data>::operator=(const LinkDataMap<Data> &other)
    {
        int count_data = 0;
        for (auto pair: other)
        {
            this->data_array_[count_data++] = pair;
        }

        return *this;
    }
}

#endif //_ROBOTLIB_LINK_DATA_MAP_TPP_