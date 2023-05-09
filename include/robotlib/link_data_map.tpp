#ifndef _ROBOTLIB_LINK_DATA_MAP_TPP_
#define _ROBOTLIB_LINK_DATA_MAP_TPP_

#include "link_data_map.hpp"

namespace robotlib
{    
    template <class Data>
    LinkDataMap<Data>::~LinkDataMap(){}

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const RobotBase* robot) 
        : DataMap<Link, Data>(DataHelper::getRobotNumLinks(robot))
    {
        int count_data = 0;
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getLinks()))
            {
                this->data_array_[count_data++] = this->createPair(key);
            }
        }
    }

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const RobotBase* robot, const Data &data) 
        : DataMap<Link, Data>(DataHelper::getRobotNumLinks(robot))
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

    template <class Data>
    LinkDataMap<Data>::LinkDataMap(const LinkDataMap &other)
        : DataMap<Joint, Data>(other.getSize())
    {
        int count_data = 0;
        for (auto pair: other)
        {
            this->data_array_[count_data++] = this->createPair(pair.key_);
        }
    }

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