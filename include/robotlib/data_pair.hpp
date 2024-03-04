#ifndef _ROBOTLIB_DATA_PAIR_HPP_
#define _ROBOTLIB_DATA_PAIR_HPP_

namespace robotlib
{
    /*!
     * @brief DataPair class.
     * @details
     * This class is introduced because of the private constructors of the data structures defined in Robotlib.
    */
    template <class Key, class Data>
    class DataPair
    {
        friend class RobotBase;
        template <class K, class D> friend class DataMap;

    public:

        /*!
         * @brief Copy constructor.
         * @param[in] pair DataPair to copy.
         */
        DataPair(const DataPair& pair);

        /*!
         * @brief Default destructor.
         */
        ~DataPair() = default;

        /*!
         * @brief Assignment operator.
         * @param[in] pair DataPair object whose key-data pair is assigned to the object pointed by *this*.
         * @return reference to the DataPair object pointed by *this*.
         * @details The key does not change, only the data is copied.
         */
        virtual DataPair& operator=(const DataPair& pair);

        /*!
         * @brief Assignment operator.
         * @param[in] pair DataPair object whose key-data pair is assigned to the object pointed by *this*.
         * @return reference to the DataPair object pointed by *this*.
         * @details The key does not change, only the data is copied.
         */
        virtual DataPair& operator=(const std::shared_ptr<DataPair> pair);

        /*!
         * @brief Assignment operator.
         * @param[in] data Data object copied inside *this*.
         * @return reference to the DataPair object pointed by *this*.
         * @details The data is copied.
         */
        virtual DataPair& operator=(const Data& data);

        /*!
         * @brief Addition operator.
         * @param[in] data Data object to sum.
         * @return The DataPair object with the result.
         */
        virtual DataPair operator+(const Data& data);
    
        /*!
         * @brief Addition assignment operator.
         * @param[in] data Data object to sum.
         * @return Reference to the DataPair object pointed by *this*.
         */
        virtual DataPair& operator+=(const Data& data);

        /*!
         * @brief Subtraction operator.
         * @param[in] data Data object to subtract.
         * @return The DataPair with the result.
         */
        virtual DataPair operator-(const Data& data);
    
        /*!
         * @brief Subtraction assignment operator.
         * @param[in] data Data object to subtract.
         * @return Reference to the DataPair object pointed by *this*.
         */
        virtual DataPair& operator-=(const Data& data);

        /*!
         * @brief Method to retrieve the key information of *this*.
         * @return Constant reference to the key of *this*.
         */
        const Key& getKey() const;

        /*!
         * @brief Method to retrieve the data information of *this*.
         * @return Reference to the data of *this*.
         */
        Data& getData();

        /*!
         * @brief Method to retrieve the data information of *this*.
         * @return reference to the data of *this*.
         * @details Implementation for constant objects.
         */
        const Data& getData() const;

    private:
        /*!
         * @brief Constructor.
         * @param[in] key reference to map key.
         * @param[in] data pointer to the data to be associated to the key.
         */
        DataPair(const Key& key, const std::shared_ptr<Data> data);

        /*!
         * @brief Constructor.
         * @param[in] key reference to map key.
         * @param[in] data data to be associated to the key.
         */
        DataPair(const Key& key, const Data& data);

        /*!
         * @brief Constructor.
         * @param[in] key pointer to the key.
         * @param[in] data pointer to the data to be associated to the key.
         */
        DataPair(const std::shared_ptr<Key> key, const std::shared_ptr<Data> data);

        /*!
         * @brief Constructor.
         * @param[in] key pointer to the key.
         * @param[in] data data to be associated to the key.
         */
        DataPair(const std::shared_ptr<Key> key, const Data& data);

        //! Key to which associate a data.
        std::shared_ptr<const Key> key_;

        //! Data to be associated to a key.
        std::shared_ptr<Data> data_;
    };
}

#include "data_pair.tpp"

#endif //_ROBOTLIB_DATA_PAIR_HPP_