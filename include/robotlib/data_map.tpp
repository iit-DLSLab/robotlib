#ifndef _ROBOTLIB_DATA_MAP_TPP_
#define _ROBOTLIB_DATA_MAP_TPP_

#include "data_map.hpp"
#include <assert.h>
#include <iostream>

namespace robotlib
{
    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const ContainerBase<Key>& keys,  const Data& data)
        : num_data_(keys.length())
    {
        int count_data = 0;
        for(auto& key : keys)
        {
            this->data_array_[count_data++] = std::shared_ptr<DataPair<Key, Data>>(new DataPair<Key, Data>(key, data));
        }   
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const ContainerBase<Key>& keys,  const std::vector<Data>& data) 
        : num_data_(keys.length())
    {
        int count_data = 0;
        for(auto& key : keys)
        {
            this->data_array_[count_data] = std::shared_ptr<DataPair<Key, Data>>(new DataPair<Key, Data>(key, data[count_data]));
            count_data++;
        }   
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const ContainerBase<std::shared_ptr<Key>> keys,  const Data& data)
        : num_data_(keys.length())
    {
        int count_data = 0;
        for(auto& key : keys)
        {
            this->data_array_[count_data++] = std::shared_ptr<DataPair<Key, Data>>(new DataPair<Key, Data>(key, data));
        }   
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const ContainerBase<std::shared_ptr<Key>> keys,  const std::vector<Data>& data) 
        : num_data_(keys.length())
    {
        int count_data = 0;
        for(auto& key : keys)
        {
            this->data_array_[count_data] = std::shared_ptr<DataPair<Key, Data>>(new DataPair<Key, Data>(key, data[count_data]));
            count_data++;
        }   
    }

    template <class Key, class Data>
    DataMap<Key, Data>::DataMap(const DataMap<Key, Data>& data)
        : num_data_(data.size())
    {
        int count_data = 0;
        for (auto& pair : data)
        {
            this->data_array_[count_data++] = std::shared_ptr<DataPair<Key, Data>>(new DataPair<Key, Data>(pair));
        }
    }

    template <class Key, class Data>
    Iterator<DataPair<Key, Data>> DataMap<Key, Data>::begin() 
    { 
        return Iterator<DataPair<Key, Data>>(&data_array_[0]);
    }
    
    template <class Key, class Data>
    Iterator<DataPair<Key, Data>> DataMap<Key, Data>::end()
    { 
        return Iterator<DataPair<Key, Data>>(&data_array_[num_data_]);
    }

    template <class Key, class Data>
    Iterator<const DataPair<Key, Data>> DataMap<Key, Data>::begin() const 
    { 
        return Iterator<const DataPair<Key, Data>>(&data_array_[0]); 
    }
        
    template <class Key, class Data>
    Iterator<const DataPair<Key, Data>> DataMap<Key, Data>::end() const 
    { 
        return Iterator<const DataPair<Key, Data>>(&data_array_[num_data_]); 
    }

    template <class Key, class Data>
    const Data& DataMap<Key, Data>::operator[](const Key& key) const
    {
        for (auto& pair : *this)
        {
            if (pair.getKey().getName().compare(key.getName()) == 0)
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    const Data& DataMap<Key, Data>::operator[](const std::shared_ptr<const Key>& key) const
    {
        for (auto& pair : *this)
        {
            if (pair.getKey() == *key)
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    const Data& DataMap<Key, Data>::operator[](const DataPair<Key, Data>& in_pair) const
    {
        for (auto& pair : *this)
        {
            if (pair.getKey() == in_pair.getKey())
                return *pair.data_;
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const Key& key)
    {
        for (auto &pair : *this)
        {
            if (pair.getKey() == key)
            {
                return *pair.data_;
            }
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const std::shared_ptr<const Key>& key)
    {
        for (auto& pair : *this)
        {
            if (pair.getKey().getName().compare(key->getName()) == 0)
            {
                return *pair.data_;
            }
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const DataPair<Key, Data>& in_pair) // q: shared_ptr or & ?
    {
        for (auto &pair : *this)
        {
            if (pair.getKey() == in_pair.getKey())
            {
                return *pair.data_;
            }
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    Data& DataMap<Key, Data>::operator[](const std::string& id)
    {
        for (auto &pair : *this)
        {
            if (pair.getKey().getName() == id)
            {
                return *pair.data_;
            }
        }
        throw std::range_error("key not found");
    }

    template <class Key, class Data>
    void DataMap<Key, Data>::copyData(const DataMap &rhs)
    {
        assert(this->size() == rhs.size());

        for(auto& data_pair : *this)
        {
            *data_pair.data_ = rhs[data_pair];
        }
    }

    template <class Key, class Data>
    void DataMap<Key, Data>::assignAll(const Data &value)
    {
        for(auto& data_pair : *this)
        {
            *data_pair.data_ = value;
        }
    }

    template <class Key, class Data>
    std::vector<Data> DataMap<Key, Data>::tovec_() const
    {
        std::vector<Data> out(this->size());

        for (auto& data_pair : *this)
        {
            out.push_back(data_pair.getData());
        }
        return out;
    }

    template <class Key, class Data>    
    DataMap<Key, Data>& DataMap<Key, Data>::operator=(const DataMap<Key, Data> &rhs)
    {
        if (&rhs != this)
        {
            copyData(rhs);
        }
        return *this;
    }

    template <class Key, class Data>    
    DataMap<Key, Data>& DataMap<Key, Data>::operator=(const std::vector<Data> &rhs)
    {
        
        assert(this->size() == (int) rhs.size());

        unsigned int data_count{0};
        for(auto& pair : *this)
        {
            *pair.data_ = rhs[data_count++];
        }
        return *this;
    }

    template <class Key, class Data>
    DataMap<Key, Data>& DataMap<Key, Data>::operator=(const Data &value)
    {
        assignAll(value);
        return *this;
    }

    template <class Key, class Data>
    DataMap<Key, Data> DataMap<Key, Data>::operator+(const DataMap& rhs)
    {
        DataMap<Key, Data> out(*this);

        for (auto& data_pair : *this)
        {
            out[data_pair] += rhs[data_pair];
        }
        return out;
    }

    template <class Key, class Data>
    DataMap<Key, Data>& DataMap<Key, Data>::operator+=(const DataMap& rhs)
    {
        for (auto& data_pair : *this)
        {
            data_pair += rhs[data_pair];
        }
        return *this;
    }

    template <class Key, class Data>
    DataMap<Key, Data> DataMap<Key, Data>::operator-(const DataMap& rhs)
    {
        DataMap<Key, Data> out(*this);

        for (auto& data_pair : *this)
        {
            out[data_pair] -= rhs[data_pair];
        }
        return out;
    }

    template <class Key, class Data>
    DataMap<Key, Data>& DataMap<Key, Data>::operator-=(const DataMap& rhs)
    {
        for (auto& data_pair : *this)
        {
            data_pair -= rhs[data_pair];
        }
        return *this;
    }
    

    template <class Key, class Data>
    bool DataMap<Key, Data>::operator==(const DataMap<Key, Data>& rhs) const
    {
        if(this->size() != rhs.size())
            return false;
        
        for (auto& data_pair : *this)
        {
            if(data_pair.getData() != rhs[data_pair])
                return false;
        }

        return true;
    }

    template <class Key, class Data>
    bool DataMap<Key, Data>::operator!=(const DataMap<Key, Data>& rhs) const
    {
        return !this->operator==(rhs);
    }

    template <class Key, class Data>
    unsigned int DataMap<Key, Data>::size() const { 
        return num_data_;
    }       
}

#endif //_ROBOTLIB_DATA_MAP_TPP_