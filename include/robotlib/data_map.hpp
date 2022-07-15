#ifndef _ROBOTLIB_DATA_MAP_HPP_
#define _ROBOTLIB_DATA_MAP_HPP_

#include "utils.hpp"

namespace robotlib
{

    template <class Key, class Data>
    class DataMap
    {
    private:
        class Pair
        {
        public:
            friend class RobotBase;
            friend class DataMap;

            virtual Pair &operator=(const Pair &rhs)
            {
                this->key_ = rhs.key_;
                this->data_ = rhs.data_;
                return *this;
            }

            std::shared_ptr<Key> key_;
            Data data_;

            ~Pair(){};

        private:
            Pair(const std::shared_ptr<Key> key, const Data &data) : key_(key), data_(data){};
            Pair(){};
        };

    public:
        friend class RobotBase;
        virtual ~DataMap();

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

        virtual const int getSize() const;

    protected:
        DataMap(const int num_data);
        DataMap(); //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!

        virtual void init(const DataMap &data);

        const Pair createPair(const std::shared_ptr<Key> key, const Data &data) const; //shared_pointers?}
        Pair createPair(const std::shared_ptr<Key> key, const Data &data); //shared_pointers?}

        int num_data_;
        Pair *data_;
    };
}

#include "data_map.tpp"

#endif //_ROBOTLIB_DATA_MAP_HPP_