#ifndef _ROBOTLIB_LEG_DATA_MAP_HPP_
#define _ROBOTLIB_LEG_DATA_MAP_HPP_

#include "limb_base.hpp"
#include "data_map.hpp"

namespace robotlib
{        
    template <class Data>
    class LegDataMap : public DataMap<LimbBase, Data>
    {
        friend class RobotBase;

    public:
        LegDataMap(const LegDataMap&);
        ~LegDataMap();
        
        LegDataMap<Data>& operator=(const LegDataMap<Data>&);
        LegDataMap<Data>& operator=(const std::vector<Data>&);

        using DataMap<LimbBase, Data>::operator=;

        operator std::vector<Data>() const;

        LegDataMap<Data>& operator<< (Data);
        LegDataMap<Data>& operator, (Data);

        /// TODO: Print in new line if data is a vector, matrix, etc... in same line of leg name (as for JointState) if data is a single value
        void print() const;

    protected:

        LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>>); //TODO: remove it, leave only the constructor with data
        LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>>, const Data &);

    private:
        Iterator<typename DataMap<LimbBase, Data>::Pair> dataIt;
    };
}

#include "leg_data_map.tpp"

#endif //_ROBOTLIB_LEG_DATA_MAP_HPP_