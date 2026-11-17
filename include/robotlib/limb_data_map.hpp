#ifndef _ROBOTLIB_LIMB_DATA_MAP_HPP_
#define _ROBOTLIB_LIMB_DATA_MAP_HPP_

#include "limb.hpp"
// #include "data_map.hpp"
#include <map>

namespace robotlib
{
    template <class Data>
    using LimbDataMap =  std::map<LimbPtr, Data>;
}

// #include "limb_data_map.tpp"

#endif //_ROBOTLIB_LIMB_DATA_MAP_HPP_