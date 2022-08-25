#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

#include <memory>

#include "leg_data_map.hpp"
#include "joint_data_map.hpp"
#include "joint.hpp"

namespace robotlib
{
    class JointState : public LegDataMap<std::shared_ptr<JointDataMap<double>>>
    {
    public:
        friend class RobotBase;
        using LegDataMap<std::shared_ptr<JointDataMap<double>>>::operator[];

        double &operator[](const std::shared_ptr<Joint> joint);
        const double &operator[](const std::shared_ptr<Joint> joint) const;
        JointState &operator=(const double data);
        JointState &operator=(const JointState &other);

        void setZero();
        int size() const;

        /**
         * @brief Return the max value of the joint state
         * @return double
         */
        double max();

        /**
         * @brief Return the min value of the joint state
         * @return double
         */
        double min();

        void print();

        std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg);
        const std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) const;

        ~JointState();

    private:

        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs);
    };

} //namespace robotlib

#endif //_ROBOTLIB_JOINT_STATE_HPP_