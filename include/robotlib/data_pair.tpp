#ifndef _ROBOTLIB_DATA_PAIR_TPP_
#define _ROBOTLIB_DATA_PAIR_TPP_

#include "data_pair.hpp"
#include <memory>

namespace robotlib
{
    template <class Key, class Data>
    DataPair<Key, Data>::DataPair(const DataPair& pair) 
        : key_(pair.key_)
        , data_(std::shared_ptr<Data>(new Data(*pair.data_)))
    {};

    // template <class Key, class Data>
    // DataPair<Key, Data>::DataPair(const Key* key, const std::shared_ptr<Data> data) 
    //     : key_(std::shared_ptr<Key>(key))
    //     , data_(std::shared_ptr<Data>(new Data(*data)))
    // {}

    // template <class Key, class Data>
    // DataPair<Key, Data>::DataPair(const Key* key, const Data& data) 
    //     : key_(std::shared_ptr<Key>(key))
    //     , data_(std::shared_ptr<Data>(new Data(data)))
    // {};

    template <class Key, class Data>
    DataPair<Key, Data>::DataPair(const std::shared_ptr<Key> key, const std::shared_ptr<Data> data) 
        : key_(key)
        , data_(std::shared_ptr<Data>(new Data(*data)))
    {};

    template <class Key, class Data>
    DataPair<Key, Data>::DataPair(const std::shared_ptr<Key> key, const Data& data) 
        : key_(key)
        , data_(std::shared_ptr<Data>(new Data(data)))
    {};
        
    template <class Key, class Data>
    DataPair<Key, Data>& DataPair<Key, Data>::operator=(const DataPair& rhs)
    {
        this->data_ = rhs.data_;
        return *this;
    }

    template <class Key, class Data>
    DataPair<Key, Data>& DataPair<Key, Data>::operator=(const std::shared_ptr<DataPair> rhs)
    {
        this->data_ = rhs->data_;
        return *this;
    }

    template <class Key, class Data>
    DataPair<Key, Data>& DataPair<Key, Data>::operator=(const Data& rhs)
    {
        *this->data_ = rhs;
        return *this;
    }

    template <class Key, class Data>
    DataPair<Key, Data> DataPair<Key, Data>::operator+(const Data& rhs)
    {
        DataPair out(*this);
        *out.data_ += rhs;
        return out;
    }
    
    template <class Key, class Data>
    DataPair<Key, Data>& DataPair<Key, Data>::operator+=(const Data& rhs)
    {
        *this->data_ += rhs;
        return *this;
    }

    template <class Key, class Data>
    DataPair<Key, Data> DataPair<Key, Data>::operator-(const Data& rhs)
    {
        DataPair out(*this);
        *out.data_ -= rhs;
        return out;
    }

    template <class Key, class Data>    
    DataPair<Key, Data>& DataPair<Key, Data>::operator-=(const Data& rhs)
    {
        *this->data_ -= rhs;
        return *this;
    }

    template <class Key, class Data>
    const Key& DataPair<Key, Data>::getKey() const 
    {
        return *key_;
    }
    
    template <class Key, class Data>
    Data& DataPair<Key, Data>::getData() 
    {
        return *data_;
    }

    template <class Key, class Data>
    const Data& DataPair<Key, Data>::getData() const
    {
        return *data_;
    }
}

#endif //_ROBOTLIB_DATA_PAIR_TPP_