#ifndef _ROBOTLIB_DATA_MAP_HPP_
#define _ROBOTLIB_DATA_MAP_HPP_

#include "utils/container_base.hpp"
#include "limb_base.hpp"
#include <iostream>
#include <vector>
namespace robotlib
{
    class RobotBase;

    class DataHelper
    {
    public:
        friend class RobotBase;

        static int getRobotNumLinks(const RobotBase*);
        static int getRobotNumJoints(const RobotBase*);
        static const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs(const RobotBase*);
    };

    /*!
     * @brief DataMap class.
     * @details
     * This templated class is a wrap around a list of Pair objects, storing a data for each key.
     * @tparam Key class of the keys to which associate data.
     * @tparam Data class of the data associated to keys.
     */
    template <class Key, class Data>
    class DataMap
    {
    friend class JointState;
    template <class T> friend class LegDataMap;
    
    private:
        /*!
         * @brief Pair class.
         * @details
         * This class is introduces because of the private constructors of the data structures defined in Robotlib.
         */
        class Pair
        {
        public:
            //! RobotBase is a friend class to let it use the private costructor of the Pair class.
            friend class RobotBase;
            friend class DataMap;
            friend class JointState;
            template <class T> friend class LegDataMap;

            /*!
             * @brief Destructor.
             */
            ~Pair(){};

            /*!
             * @brief Equal operator.
             * @param[in] pair Pair object whose key-data pair is assigned to the object pointed by *this*.
             * @return reference to the Pair object pointed by *this*.
            */
            virtual Pair &operator=(const Pair &rhs)
            {
                this->key_ = rhs.key_;
                if((this->data_ != nullptr) && (rhs.data_ != nullptr))
                    *this->data_ = *rhs.data_;

                return *this;
            }

            virtual Pair &operator=(const std::shared_ptr<Pair> rhs)
            {
                this->key_ = rhs->key_;
                this->data_ = rhs->data_;

                return *this;
            }

            //! Key to which associate a data.
            std::shared_ptr<Key> key_;

            //! Data to be associated to a key.
            std::shared_ptr<Data> data_;

        private:
            /*!
             * @brief Constructor.
             * @param[in] key shared pointer pointing to the key.
             * @param[in] data data to be associated to the key.
             */
            Pair(const std::shared_ptr<Key> key, const Data data) 
                : key_(key)
                , data_(new Data(data))
            {};

            Pair(const std::shared_ptr<Key> key) 
                : key_(key)
                , data_(nullptr)
            {};

            /*!
             * @brief Empty constructor.
             */
            Pair() 
                : key_(nullptr)
                , data_(nullptr)
            {};
        };
    
        int getRobotNumLinks(RobotBase* robot_base);

    public:
        //! RobotBase is a friend class to let it use the private costructor of the DataMap class.
        friend class RobotBase;

        /*!
        * @brief Destructor.
        */
        virtual ~DataMap();
        // DataMap (const DataMap&);

        /*!
         * @brief Begin function to be used with iterators.
         * @return iterator object pointing to the first data of the data_ array.
         */
        virtual Iterator<Pair> begin();

        /*!
         * @brief End function to be used with iterators.
         * @return iterator object pointing to the last data of the data_ array.
         */
        virtual Iterator<Pair> end();

        /*!
         * @brief Begin function to be used with iterators.
         * @details
         * Implementation for constant objects.
         * @return iterator object pointing to the first data of the data_ array.
         */
        virtual Iterator<const Pair> begin() const;

        /*!
         * @brief End function to be used with iterators.
         * @details
         * Implementation for constant objects.
         * @return iterator object pointing to the last data of the data_ array.
         */
        virtual Iterator<const Pair> end() const;

        /*!
         * @brief Square brackets operator.
         * @details
         * This function allows to access to the data associated to the key in input.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual Data &operator[](const std::shared_ptr<Key> key);

        /*!
         * @brief Square brackets operator.
         * @details
         * This function allows to access to the data associated to the key in input.
         * 
         * Implementation for constant objects.
         * @param[in] key shared pointer pointing to the key.
         * @return reference to the data associated to the key.
         */
        virtual const Data &operator[](const std::shared_ptr<Key> key) const;

        /*!
         * @brief Square brackets operator.
         * @details
         * This function allows to access to the data associated to the key whose name is given in input.
         * @param[in] key_name name of the key.
         * @return reference to the data associated to the key.
         */
        virtual Data &operator[](const std::string &key_name);

        /*!
         * @brief Square brackets operator.
         * @details
         * It allows to access to the data associated to the key whose name is given in input.
         * 
         * Implementation for constant objects.
         * @param[in] key_name name of the key.
         * @return reference to the data associated to the key.
         */
        virtual const Data &operator[](const std::string &key_name) const;

        /*!
         * @brief Copying keys and data from another DataMap object.
         * @param[in] data_map DataMap object.
         */
        virtual void copydata(const DataMap &rhs);

        /*!
         * @brief Assigning a value to all the keys of the DataMap object.
         * @param[in] value value to be assigned to all the keys.
         */
        virtual void assignAll(const Data &value);

        /*!
         * @brief Equal operator.
         * @param[in] data_map DataMap object whose key-data pairs are assigned to the object pointed by *this*.
         * @return reference to the DataMap object pointed by *this*.
         */
        virtual DataMap &operator=(const DataMap &rhs);

        /*!
         * @brief Equal operator.
         * @details
         * It assigns the value in input to all the keys.
         * @param[in] value value to be assigned to all the keys of the DataMap object.
         * @return reference to the DataMap object pointed by *this*.
         */
        virtual DataMap &operator=(const Data &defaultValue);

        virtual DataMap &operator=(const std::vector<Data>&);

        /*!
         * @brief Get number of pairs stored by the DataMap object.
         * @return number of pairs stored by the DataMap object.
         */
        virtual int getSize() const;

    protected:
        /*!
         * @brief DataMap constructor.
         * @param[in] num_data number of pairs to be stored.
         */
        DataMap(const int num_data);

        virtual void init(const DataMap &data);

        std::shared_ptr<DataMap::Pair> createPair(const Pair& pair);

        /*!
         * @brief Create a Pair object.
         * @param[in] key shared pointer pointing to the key.
         * @param[in] data data to be associated to the key.
         * @return Pair object.
         */
        std::shared_ptr<DataMap::Pair> createPair(const std::shared_ptr<Key>, const Data& data);
        std::shared_ptr<DataMap::Pair> createPair(const std::shared_ptr<Key>);

        //! Pointer pointing to the array of pairs.
        Pair *data_array_;

        //! Number of pairs.
        int num_data_{};
    };
}

#include "data_map.tpp"

#endif //_ROBOTLIB_DATA_MAP_HPP_