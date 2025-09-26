#ifndef _ROBOTLIB_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_HPP_

#include "link.hpp"
#include <map>
namespace robotlib
{    
    template <class Data>
    using LinkDataMap =  std::map<LinkPtr, Data>;
}

// #include "link_data_map.tpp"

#endif //_ROBOTLIB_LINK_DATA_MAP_HPP_