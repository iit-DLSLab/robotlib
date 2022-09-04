#ifndef _ROBOTLIB_DATA_MAP_TPP_
#define _ROBOTLIB_DATA_MAP_TPP_

#include "data_map.hpp"
#include <assert.h>
#include <iostream>

namespace robotlib
{

    template <class Key, class Data>
    DataMap<Key, Data>::~DataMap()
    {
        if(data_array_ != nullptr)
            delete[] data_array_;
    }

    template <class Key, class Data>
    Iterator<typename DataMap<Key, Data>::Pair> DataMap<Key, Data>::begin() 
    { 
        return Iterator<Pair>(&data_array_[0]);
    }
    
    template <class Key, class Data>
    Iterator<typename DataMap<Key, Data>::Pair> DataMap<Key, Data>::end()
    { 
        return Iterator<Pair>(&data_array_[num_data_]);
    }

    template <class Key, class Data>
    Iterator<const typename DataMap<Key, Data>::Pair> DataMap<Key, Data>::begin() const 
    { 
        return Iterator<const Pair>(&data_array_[0]); 
    }
        
    template <class Key, class Data>
    Iterator<const typename DataMap<Key, Data>::Pair> DataMap<Key, Data>::end() const 
    { 
        return Iterator<const Pair>(&data_array_[num_data_]); 
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const std::shared_ptr<Key> key) // q: shared_ptr or & ?
    {
        for (Pair &pair : *this)
        {
            if (pair.key_->getName().compare(key->getName()) == 0)
            {
                return *pair.data_;
            }
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    const Data& DataMap<Key, Data>::operator[](const std::shared_ptr<Key> key) const // q: shared_ptr or & ?
    {
        for (auto &pair : *this)
        {
            if (pair.key_->getName().compare(key->getName()) == 0)
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const std::string &key_name)
    {
        for (Pair &pair : *this)
        {
            if (pair.key_->getName().compare(key_name) == 0)
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    const Data& DataMap<Key, Data>::operator[](const std::string &key_name) const
    {
        for (auto &pair : *this)
        {
            if (pair.key_->getName().compare(key_name) == 0)
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    void DataMap<Key, Data>::copydata(const DataMap &rhs)
    {
        assert(this->getSize() == rhs.getSize());

        for (auto i{0}; i < num_data_; i++)
        {
            data_array_[i].key_ = rhs.data_array_[i].key_;
            *data_array_[i].data_ = *rhs.data_array_[i].data_;
        }
    }

    template <class Key, class Data>
    void DataMap<Key, Data>::assignAll(const Data &value)
    {
        for (auto i{0}; i < num_data_; i++)
        {
            *(data_array_[i].data_) = value;
        }
    }

    template <class Key, class Data>    
    DataMap<Key, Data>& DataMap<Key, Data>::operator=(const DataMap<Key, Data> &rhs)
    {
        if (&rhs != this)
        {
            copydata(rhs);
        }
        return *this;
    }

    // template <class Key, class Data>    
    // DataMap<Key, Data>::DataMap(const DataMap<Key, Data> &rhs)
    // {
    //     if (&rhs != this)
    //     {
    //         copydata(rhs);
    //     }
    // }


    template <class Key, class Data>
    DataMap<Key, Data>& DataMap<Key, Data>::operator=(const Data &defaultValue)
    {
        assignAll(defaultValue);
        return *this;
    }

    template <class Key, class Data>
    int DataMap<Key, Data>::getSize() const { 
        return num_data_;
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const int num_data) : num_data_(num_data)
    {
        this->data_array_ = new Pair[this->num_data_];
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap() : num_data_(0), data_array_(nullptr) //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
    {
    }

    template <class Key, class Data>
    void DataMap<Key, Data>::init(const DataMap<Key, Data> &data)
    {
        num_data_ = data.getSize();

        if (data_array_ != nullptr)
        {
            delete[] data_array_;
        }

        data_array_ = new Pair[num_data_];

        copydata(data);
    }

    // template <class Key, class Data>
    // const typename DataMap<Key, Data>::Pair DataMap<Key, Data>::createPair(const std::shared_ptr<Key> key, const Data &data) const 
    // { 
    //     return Pair(key, data); 
    // }
    
    template <class Key, class Data>
    typename DataMap<Key, Data>::Pair DataMap<Key, Data>::createPair(const std::shared_ptr<Key> key, const Data &data) 
    { 
        return Pair(key, data);
    }  

    template <class Key, class Data>
    typename DataMap<Key, Data>::Pair DataMap<Key, Data>::createPair(const std::shared_ptr<Key> key) 
    { 
        return Pair(key);
    }            
}

#endif //_ROBOTLIB_DATA_MAP_TPP_