
#ifndef _ROBOTLIB_LEG_DATA_MAP_TPP_
#define _ROBOTLIB_LEG_DATA_MAP_TPP_

#include "leg_data_map.hpp"
#include <iostream>

namespace robotlib
{        
    
    template <class Data>
    LegDataMap<Data>::~LegDataMap(){}

    template <class Data>
    void LegDataMap<Data>::print()
    {
        std::cout << "LegDataMap [Name - Value]" << std::endl;
        std::cout << "-------------------------" << std::endl;

        for (auto leg_pair : *this)
        {
            std::cout << leg_pair.key_->getName() << " - " << leg_pair.data_ << std::endl;
        }
    }

    template <class Data>
    LegDataMap<Data>::LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs) 
        : DataMap<LimbBase, Data>(legs->size()) //TODO: remove it, leave only the constructor with data
    {
        int count_data = 0;
        for (auto key : *legs)
        {
            this->data_array_[count_data++] = this->createPair(key);
        }
    }

    template <class Data>
    LegDataMap<Data>::LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs, const Data &data) 
        : DataMap<LimbBase, Data>(legs->size())
    {
        int count_data = 0;
        for (auto key : *legs)
        {
            this->data_array_[count_data++] = this->createPair(key, data);
        }
    }

    template <class Data>
    LegDataMap<Data>::LegDataMap(const LegDataMap &other)
        : DataMap<LimbBase, Data>(other.getSize())
    {
        int count_data = 0;
        for (auto pair: other)
        {
            this->data_array_[count_data++] = this->createPair(pair.key_);
        }
    }

    template <class Data>
    LegDataMap<Data> &LegDataMap<Data>::operator=(const LegDataMap<Data> &other)
    {
        int count_data = 0;
        for (auto pair: other)
        {
            this->data_array_[count_data++] = pair;
        }

        return *this;
    }
}

#endif //_ROBOTLIB_LEG_DATA_MAP_TPP_