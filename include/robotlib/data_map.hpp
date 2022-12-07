#ifndef _ROBOTLIB_DATA_MAP_HPP_
#define _ROBOTLIB_DATA_MAP_HPP_

#include "utils.hpp"
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
    
    template <class Key, class Data>
    class DataMap
    {
    friend class JointState;
    template <class T> friend class LegDataMap;
    
    private:
        class Pair
        {
        public:
            friend class RobotBase;
            friend class DataMap;
            friend class JointState;
            template <class T> friend class LegDataMap;

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

            std::shared_ptr<Key> key_;
            std::shared_ptr<Data> data_;

            ~Pair(){};

        private:
            Pair(const std::shared_ptr<Key> key, const Data data) 
                : key_(key)
                , data_(new Data(data))
            {};

            Pair(const std::shared_ptr<Key> key) 
                : key_(key)
                , data_(nullptr)
            {};

            Pair() 
                : key_(nullptr)
                , data_(nullptr)
            {};
        };
    
        int getRobotNumLinks(RobotBase *);

    public:
        friend class RobotBase;
        virtual ~DataMap();
        // DataMap (const DataMap&);

        virtual Iterator<Pair> begin();
        virtual Iterator<Pair> end();

        virtual Iterator<const Pair> begin() const;
        virtual Iterator<const Pair> end() const;

        virtual Data &operator[](const std::shared_ptr<Key> key); // q: shared_ptr or & ?
        virtual const Data &operator[](const std::shared_ptr<Key> key) const; // q: shared_ptr or & ?

        virtual Data &operator[](const std::string &key_name);

        virtual const Data &operator[](const std::string &key_name) const;

        virtual void copydata(const DataMap &rhs);

        virtual void assignAll(const Data &value);

        virtual DataMap &operator=(const DataMap &rhs);

        virtual DataMap &operator=(const Data &defaultValue);

        virtual DataMap &operator=(const std::vector<Data>&);

        virtual int getSize() const;

    protected:
        DataMap(const int num_data);
        DataMap(); //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!

        virtual void init(const DataMap &data);

        std::shared_ptr<DataMap::Pair> createPair(const Pair&);
        std::shared_ptr<DataMap::Pair> createPair(const std::shared_ptr<Key>, const Data&);
        std::shared_ptr<DataMap::Pair> createPair(const std::shared_ptr<Key>);

        int num_data_;
        Pair *data_array_;
    };
}

#include "data_map.tpp"

#endif //_ROBOTLIB_DATA_MAP_HPP_