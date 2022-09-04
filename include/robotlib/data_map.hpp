#ifndef _ROBOTLIB_DATA_MAP_HPP_
#define _ROBOTLIB_DATA_MAP_HPP_

#include "utils.hpp"
#include "limb_base.hpp"
#include <iostream>
namespace robotlib
{
    class RobotBase;

    class DataHelper
    {
    public:
        friend class RobotBase;

        static int getRobotNumLinks(std::shared_ptr<RobotBase>);
        static int getRobotNumJoints(std::shared_ptr<RobotBase>);
        static const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs(std::shared_ptr<RobotBase>);
    };
    
    template <class Key, class Data>
    class DataMap
    {
    friend class JointState;
    private:
        class Pair
        {
        public:
            friend class RobotBase;
            friend class DataMap;
            friend class JointState;

            virtual Pair &operator=(const Pair &rhs)
            {
                this->key_ = rhs.key_;
                this->data_ = rhs.data_;
                return *this;
            }

            std::shared_ptr<Key> key_;
            std::shared_ptr<Data> data_;

            ~Pair(){};

        private:
            Pair(const std::shared_ptr<Key> key, const std::shared_ptr<Data> data) : key_(key), data_(data){};
            Pair(){};
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

        virtual int getSize() const;

    protected:
        DataMap(const int num_data);
        DataMap(); //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!

        virtual void init(const DataMap &data);

        const Pair createPair(const std::shared_ptr<Key> key, const std::shared_ptr<Data> data) const; //shared_pointers?}
        Pair createPair(const std::shared_ptr<Key> key, const std::shared_ptr<Data> data); //shared_pointers?}

        int num_data_;
        Pair *data_array_;
    };
}

#include "data_map.tpp"

#endif //_ROBOTLIB_DATA_MAP_HPP_