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
    class JointState : public LegDataMap<std::shared_ptr<JointDataMap<double>>>
    {
    public:
        friend class RobotBase;
        using LegDataMap<std::shared_ptr<JointDataMap<double>>>::operator[];

        double &operator[](const std::shared_ptr<Joint> joint);
        const double &operator[](const std::shared_ptr<Joint> joint) const;
        JointState &operator=(const double data);
        JointState &operator=(const std::vector<double> data);
        // JointState &operator=(const JointState &other);

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

        std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg);
        const std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) const;

        ~JointState();
        JointState(std::shared_ptr<JointState>);

    private:

        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs);
        
        Iterator<DataMap<Joint, double>::Pair> jointIt;
    };

} //namespace robotlib

robotlib::JointState operator*(const double&, const robotlib::JointState&);

#endif //_ROBOTLIB_JOINT_STATE_HPP_