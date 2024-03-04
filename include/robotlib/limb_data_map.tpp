
#ifndef _ROBOTLIB_LIMB_DATA_MAP_TPP_
#define _ROBOTLIB_LIMB_DATA_MAP_TPP_

#include "limb_data_map.hpp"
#include <iostream>

namespace robotlib
{        
    
    template <class Data>
    LimbDataMap<Data>::~LimbDataMap(){}

    template <class Data>
    LimbDataMap<Data>::LimbDataMap(const ContainerBase<LimbBase>& limbs, const Data& data) 
        : DataMap<LimbBase, Data>(limbs, data)
        // , dataIt(nullptr)
    {}

    template <class Data>
    LimbDataMap<Data>::LimbDataMap(const ContainerBase<LimbBase>& limbs, const std::vector<Data>& data) 
        : DataMap<LimbBase, Data>(limbs, data)
        // , dataIt(nullptr)
    {}

    template <class Data>
    LimbDataMap<Data>::LimbDataMap(const LimbDataMap<Data> &other)
        : DataMap<LimbBase, Data>(other)
        // , dataIt(nullptr)
    {}

    // template <class Data>
    // void LimbDataMap<Data>::print() const
    // {
    //     std::cout << "LimbDataMap [Name - Value]" << std::endl;
    //     std::cout << "-------------------------" << std::endl;

    //     for (auto& limb_pair : *this)
    //     {
    //         std::cout << limb_pair.getKey().getName() << " - " << limb_pair.getDataSize() << std::endl;
    //     }
    // }

    template <class Data>
    LimbDataMap<Data>& LimbDataMap<Data>::operator=(const LimbDataMap<Data>& other)
    {
        assert (this->size() == other.size());

        for (auto& pair: other)
        {
            (*this)[pair] = pair.getData();
        }
        return *this;
    }
    
    // template <class Data>
    // LimbDataMap<Data> &LimbDataMap<Data>::operator=(const ContainerBase<std::shared_ptr<Data>>& data)
    // {
    //     assert (this.size() == data.size());

    //     int i = 0;
    //     for (auto& pair : *this)
    //     {
    //         pair = data[i++];
    //     }
    //     return *this;
    // }

    // template <class Data>
    // LimbDataMap<Data>::operator std::vector<Data>() const
    // {
    //     std::vector<Data> out;

    //     out.resize(this->getSize());

    //     int i = 0;
    //     for (auto &pair : *this)
    //     {
    //         out[i++] = *(pair.data_);
    //     }

    //     return out;
    // }

    // template <class Data>
    // LimbDataMap<Data>& LimbDataMap<Data>::operator<< (Data val)
    // {
    //     dataIt = this->begin();

    //     *(dataIt->data_) = val;

    //     return *this;
    // }

    // template <class Data>
    // LimbDataMap<Data>&  LimbDataMap<Data>::operator, (Data val)
    // {
    //     for (auto limb_pair = this->begin(); limb_pair !=  this->end(); limb_pair++)
    //     {
    //         if(limb_pair == dataIt)
    //         {
    //             limb_pair++;
    //             if(limb_pair != this->end())
    //             {
    //                 dataIt = limb_pair;
    //                 *(dataIt->data_) = val;
    //                 return *this;
    //             }
    //             else
    //             {
    //                 throw std::range_error("Number of values bigger than the data map");
    //             }
    //         }
    //     }
    //     throw std::range_error("Number of values bigger than the data map");
    // }
}

#endif //_ROBOTLIB_LIMB_DATA_MAP_TPP_