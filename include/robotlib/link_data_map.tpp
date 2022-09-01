#ifndef _ROBOTLIB_LINK_DATA_MAP_TPP_
#define _ROBOTLIB_LINK_DATA_MAP_TPP_

#include "link_data_map.hpp"

namespace robotlib
{    
    template <class Data>
    LinkDataMap<Data>::~LinkDataMap(){}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(std::shared_ptr<RobotBase> robot) : DataMap<Link, Data>(DataHelper::getRobotNumLinks(robot))
    {
        int count_data = 0;
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getLinks()))
            {
                this->data_array_[count_data++] = this->createPair(key, nullptr);
            }
        }
    }

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(std::shared_ptr<RobotBase> robot, const std::shared_ptr<Data> data) : DataMap<Link, Data>(DataHelper::getRobotNumLinks(robot))
    {
        int count_data = 0;
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getLinks()))
            {
                this->data_array_[count_data++] = this->createPair(key, data);
            }
        }
    }
}

#endif //_ROBOTLIB_LINK_DATA_MAP_TPP_