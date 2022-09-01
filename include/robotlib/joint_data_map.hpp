#ifndef _ROBOTLIB_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_HPP_

#include <memory>
#include "limb_base.hpp"

namespace robotlib
{
    class RobotBase;

    template <class Data>
    class JointDataMap : public DataMap<Joint, Data>
    {
        friend class RobotBase;
        friend class JointState;
    public:
        using DataMap<Joint, Data>::operator=;
        JointDataMap(const JointDataMap&);
        ~JointDataMap();

    private:
        JointDataMap(std::shared_ptr<RobotBase> robot);
        JointDataMap(std::shared_ptr<RobotBase> robot, const std::shared_ptr<Data> data);
        JointDataMap(const std::shared_ptr<LimbBase> leg);
        JointDataMap(const std::shared_ptr<LimbBase> leg, const std::shared_ptr<Data> data);
    };
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_