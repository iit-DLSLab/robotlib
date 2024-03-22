#ifndef _ROBOTLIB_DATA_MAP_HPP_
#define _ROBOTLIB_DATA_MAP_HPP_

#include "data_pair.hpp"
#include "utils/iterator.hpp"

namespace robotlib
{
    /*!
     * @brief DataMap class.
     * @details This templated class is a wrap around a list of DataPair objects, storing a data for each key.
     * @tparam Key class of the keys to which associate data.
     * @tparam Data class of the data associated to keys.
     */
    template <class Key, class Data>
        requires requires (Key& a) {
            {a.getName()} -> std::same_as<const std::string&>;
        }
    class DataMap
    {
    public:

        /*!
         * @brief Copy constructor.
         * @param[in] data data map to copy.
         */
        DataMap(const DataMap& data);

        /*!
         * @brief Default destructor.
         */
        virtual ~DataMap() = default;
        
        /*!
         * @brief Begin method to be used with iterators.
         * @details Implementation for constant objects.
         * @return Iterator object pointing to the first data of the data_ array.
         */
        virtual Iterator<const DataPair<Key, Data>> begin() const;

        /*!
         * @brief End method to be used with iterators.
         * @details Implementation for constant objects.
         * @return Iterator object pointing to the last data of the data_ array.
         */
        virtual Iterator<const DataPair<Key, Data>> end() const;

        /*!
         * @brief Begin method to be used with iterators.
         * @return Iterator object pointing to the first data of the data_ array.
         */
        virtual Iterator<DataPair<Key, Data>> begin();

        /*!
         * @brief End method to be used with iterators.
         * @return Iterator object pointing to the last data of the data_ array.
         */
        virtual Iterator<DataPair<Key, Data>> end();

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * Implementation for constant objects.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual const Data& operator[](const std::shared_ptr<const Key>& key) const;

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * Implementation for constant objects.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual const Data& operator[](const Key& key) const;

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * Implementation for constant objects.
         * @param[in] pair pair containing the key.
         * @return reference to the data associated to the key.
         */
        virtual const Data& operator[](const DataPair<Key, Data>& pair) const;

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual Data& operator[](const std::shared_ptr<const Key>& key);

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual Data& operator[](const Key& key);

        /*!
         * @brief Square brackets operator.
         * @details This method allows access to the data associated to the key in input.
         * @param[in] pair pair containing the key.
         * @return reference to the data associated to the key.
         */
        virtual Data& operator[](const DataPair<Key, Data>& pair);

        /*!
         * @brief Square brackets operator.
         * @details This method allows to access to the data associated to the key in input.
         * @param[in] keyid string with key id.
         * @return reference to the data associated to the key.
         */
        virtual Data& operator[](const std::string& keyid);

        /*!
         * @brief Assignment operator.
         * @param[in] data DataMap object whose data are assigned *this*.
         * Keys do not change.
         * @return reference to the DataMap object pointed by *this*.
         */
        virtual DataMap &operator=(const DataMap &data);

        /*!
         * @brief Assignment operator.
         * @details It assigns the value to all the data inside *this*.
         * Keys do not change.
         * @param[in] value value to be assigned to data items of *this*.
         * @return reference to the DataMap object pointed by *this*.
         */
        virtual DataMap& operator=(const Data& value);

        /*!
         * @brief Assignment operator.
         * @details It assigns the value to all the data inside *this*.
         * Keys do not change.
         * @param[in] data vector of Data to be assigned to each item of *this*.
         * @return reference to the DataMap object pointed by *this*.
         */
        virtual DataMap& operator=(const std::vector<Data>& data);

        /*!
         * @brief Addition operator.
         * @param[in] data DataMap object that constais the Data to subtract.
         * @return The DataPair object with the result.
         */
        virtual DataMap operator+(const DataMap& data);
        
        /*!
         * @brief Addition assignment operator.
         * @param[in] data DataMap object to sum.
         * @return Reference to the DataPair object pointed by *this*.
         */
        virtual DataMap& operator+=(const DataMap&);

        /*!
         * @brief Subtraction operator.
         * @param[in] data DataMap object that constais the Data to subtract.
         * @return The DataPair object with the result.
         */
        virtual DataMap operator-(const DataMap& data);

        /*!
         * @brief Subtraction assignment operator.
         * @param[in] data DataMap object that constais the Data to subtract.
         * @return The DataPair object pointed by *this*.
         */
        virtual DataMap& operator-=(const DataMap& data);

        /*!
         * @brief Equal comparission operator.
         * @param[in] data DataMap object that constais the Data to compare.
         * @return True if they are equal.
         */
        virtual bool operator==(const DataMap& data) const;

        /*!
         * @brief Not equal comparission operator.
         * @param[in] data DataMap object that constais the Data to compare.
         * @return True if they are not equal.
         */
        virtual bool operator!=(const DataMap& data) const;

        /*!
         * @brief Get number of pairs stored by the DataMap object.
         * @return number of pairs stored by the DataMap object.
         */
        long unsigned int size() const;

        /*!
         * @brief Get the data of the DataMap as a std vector container.
         * @return A not mapped list of the data of *this*.
         */
        std::vector<Data> tovec_() const;

    protected:

        /*!
         * @brief DataMap constructor.
         * @param[in] keys keys references to map the data.
         * @param[in] data default data value for all keys.
         */
        DataMap(const ContainerBase<Key>& keys,  const Data& data);

        /*!
         * @brief DataMap constructor.
         * @param[in] keys keys references to map the data.
         * @param[in] data vector of data values for each key.
         */
        DataMap(const ContainerBase<Key>& keys,  const std::vector<Data>& data);

        // /*!
        //  * @brief DataMap constructor.
        //  * @param[in] keys pointer to keys to map the data.
        //  * @param[in] data default data value for all keys.
        //  */
        // DataMap(const ContainerBase<std::shared_ptr<Key>> keys,  const Data& data);

        // /*!
        //  * @brief DataMap constructor.
        //  * @param[in] keys pointer to keys to map the data.
        //  * @param[in] data vector of data values for each key.
        //  */
        // DataMap(const ContainerBase<std::shared_ptr<Key>> keys,  const std::vector<Data>& data);

    private:

        /*!
         * @brief Copying keys and data from another DataMap object.
         * @param[in] data DataMap object.
         */
        void copyData(const DataMap &data);

        /*!
         * @brief Assigning a value to all the keys of the DataMap object.
         * @param[in] value value to be assigned to all the keys.
         */
        void assignAll(const Data &value);

        //! Pointer to the array of pairs.
        std::shared_ptr<DataPair<Key, Data>[]> data_array_;

        //! Number of pairs.
        long unsigned int num_data_;
    };
}
#include "data_map.tpp"

#endif //_ROBOTLIB_DATA_MAP_HPP_