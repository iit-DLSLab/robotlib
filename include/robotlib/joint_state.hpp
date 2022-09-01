#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

#include <memory>
#include <vector>

#include "leg_data_map.hpp"
#include "joint_data_map.hpp"
#include "joint.hpp"
#include <Eigen/Dense>

namespace robotlib
{
    class JointState : public LegDataMap<JointDataMap<double>>
    {
        friend class RobotBase;

    public:

        JointState(const JointState&);
        ~JointState();

        using LegDataMap<JointDataMap<double>>::operator[];

        double &operator[](const std::shared_ptr<Joint> joint);
        const double &operator[](const std::shared_ptr<Joint> joint) const;
        JointState &operator=(const double data);
        JointState &operator=(const std::vector<double> data);

        JointState operator+(const JointState &other);
        JointState &operator+=(const JointState &other);
        JointState operator-(const JointState &other);
        JointState &operator-=(const JointState &other);

        operator std::vector<double>() const;

        JointState &operator<< (double);
        JointState &operator, (double);

        Eigen::VectorXd vec_();

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

        JointDataMap<double>& getLegJointState(const std::shared_ptr<LimbBase> leg);
        const JointDataMap<double>& getLegJointState(const std::shared_ptr<LimbBase> leg) const;

    private:

        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>>);
        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>>, double);
        
        Iterator<DataMap<Joint, double>::Pair> jointIt;
    };

} //namespace robotlib

robotlib::JointState operator*(const double&, const robotlib::JointState&);

#endif //_ROBOTLIB_JOINT_STATE_HPP_