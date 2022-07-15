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
    public:
        using DataMap<Joint, Data>::operator=;
        friend class RobotBase;

        ~JointDataMap();

    private:
        JointDataMap(RobotBase *robot);
        JointDataMap(RobotBase *robot, const Data &data);
        JointDataMap(const std::shared_ptr<LimbBase> leg);
        JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data);
        JointDataMap();
    };
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_