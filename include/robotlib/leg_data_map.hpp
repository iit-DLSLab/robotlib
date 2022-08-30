#ifndef _ROBOTLIB_LEG_DATA_MAP_HPP_
#define _ROBOTLIB_LEG_DATA_MAP_HPP_

#include "limb_base.hpp"
#include "data_map.hpp"

namespace robotlib
{        
    template <class Data>
    class LegDataMap : public DataMap<LimbBase, Data>
    {
    public:
        using DataMap<LimbBase, Data>::operator=;
        friend class RobotBase;
        ~LegDataMap();

        /// TODO: Print in new line if data is a vector, matrix, etc... in same line of leg name (as for JointState) if data is a single value
        void print();

    protected:

        LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs); //TODO: remove it, leave only the constructor with data
        LegDataMap(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs, const Data &data);
        LegDataMap(std::shared_ptr<LegDataMap<Data>>);
    };
}

#include "leg_data_map.tpp"

#endif //_ROBOTLIB_LEG_DATA_MAP_HPP_