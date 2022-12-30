
#ifndef _ROBOTLIB_LEG_DATA_MAP_TPP_
#define _ROBOTLIB_LEG_DATA_MAP_TPP_

#include "leg_data_map.hpp"
#include <iostream>

namespace robotlib
{        
    
    template <class Data>
    LegDataMap<Data>::~LegDataMap(){}

    template <class Data>
    void LegDataMap<Data>::print() const
    {
        std::cout << "LegDataMap [Name - Value]" << std::endl;
        std::cout << "-------------------------" << std::endl;

        for (auto &leg_pair : *this)
        {
            std::cout << leg_pair.key_->getName() << " - " << *leg_pair.data_ << std::endl;
        }
    }

    template <class Data>
    LegDataMap<Data>::LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs) 
        : DataMap<LimbBase, Data>(legs->size()) //TODO: remove it, leave only the constructor with data
        , dataIt(nullptr)
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
        , dataIt(nullptr)
    {
        int count_data = 0;
        for (auto key : *legs)
        {
            this->data_array_[count_data++] = this->createPair(key, data);
        }
    }

    template <class Data>
    LegDataMap<Data>::LegDataMap(const LegDataMap<Data> &other)
        : DataMap<LimbBase, Data>(other.getSize())
        , dataIt(nullptr)
    {
        int count_data = 0;
        for (auto &pair: other)
        {
            this->data_array_[count_data++] = this->createPair(pair.key_, *pair.data_);
        }
    }

    template <class Data>
    LegDataMap<Data> &LegDataMap<Data>::operator=(const LegDataMap<Data> &other)
    {
        int count_data = 0;
        for (auto &pair: other)
        {
            this->data_array_[count_data].key_ = pair.key_;
            *this->data_array_[count_data++].data_ = *pair.data_;
        }

        return *this;
    }

    template <class Data>
    LegDataMap<Data> &LegDataMap<Data>::operator=(const std::vector<Data> &data)
    {
        //assert (this.size() == data.size());

        int i = 0;
        for (auto &pair : *this)
        {
            *(pair.data_) = data[i++];
        }
        return *this;
    }

    template <class Data>
    LegDataMap<Data>::operator std::vector<Data>() const
    {
        std::vector<Data> out;

        out.resize(this->getSize());

        int i = 0;
        for (auto &pair : *this)
        {
            out[i++] = *(pair.data_);
        }

        return out;
    }

    template <class Data>
    LegDataMap<Data>& LegDataMap<Data>::operator<< (Data val)
    {
        dataIt = this->begin();

        *(dataIt->data_) = val;

        return *this;
    }

    template <class Data>
    LegDataMap<Data>&  LegDataMap<Data>::operator, (Data val)
    {
        for (auto leg_pair = this->begin(); leg_pair !=  this->end(); leg_pair++)
        {
            if(leg_pair == dataIt)
            {
                leg_pair++;
                if(leg_pair != this->end())
                {
                    dataIt = leg_pair;
                    *(dataIt->data_) = val;
                    return *this;
                }
                else
                {
                    throw std::range_error("Number of values bigger than the data map");
                }
            }
        }
        throw std::range_error("Number of values bigger than the data map");
    }
}

#endif //_ROBOTLIB_LEG_DATA_MAP_TPP_