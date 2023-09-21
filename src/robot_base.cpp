#ifndef _ROBOTLIB_ROBOT_BASE_CPP_
#define _ROBOTLIB_ROBOT_BASE_CPP_

#include "robot_base.hpp"
#include "utils/utils.hpp"

namespace robotlib
{
    RobotBase::RobotBase(const std::string &name) : name_(name){};
    RobotBase::~RobotBase(){};

    // ** GET FUNCTIONS **
    const std::string RobotBase::getName() const
    {
        return name_;
    }; 

    double RobotBase::getMinJointAngle(const std::shared_ptr<Joint> joint) { return joint->getMinAngle(); };

    double RobotBase::getMaxJointAngle(const std::shared_ptr<Joint> joint) { return joint->getMaxAngle(); };

    double RobotBase::getMaxJointVelocity(const std::shared_ptr<Joint> joint) { return joint->getMaxVelocity(); };

    double RobotBase::getMaxJointEffort(const std::shared_ptr<Joint> joint) { return joint->getMaxEffort(); };

    void RobotBase::estimateFeetGRF(const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau, const Eigen::Matrix<double, 6,1>& g_b, robotlib::LegDataMap<Eigen::Vector3d>& estimated_feet_grf)
    {
        robotlib::JointState inv_dyn_tau = this->makeJointState();	// remove this NRT instatiation!
        Eigen::Matrix<double, 6, 1> wrench_base = Eigen::Matrix<double, 6, 1>::Zero();
        this->inverseDynamics(Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), g_b, q, qd, qdd, wrench_base, inv_dyn_tau);

        // robotlib::JointState stiction_positive = robot_->makeJointState();
        // robotlib::JointState stiction_negative = robot_->makeJointState();
        // robotlib::JointState torque_offset = robot_->makeJointState();
        // stiction_positive[robot_->getJoint("LF_HAA")] = 2;
        // stiction_positive[robot_->getJoint("LF_HFE")] = 2;
        // stiction_positive[robot_->getJoint("LF_KFE")] = 2;
        // stiction_positive[robot_->getJoint("RF_HAA")] = 2;
        // stiction_positive[robot_->getJoint("RF_HFE")] = 2;
        // stiction_positive[robot_->getJoint("RF_KFE")] = 2;
        // stiction_positive[robot_->getJoint("LH_HAA")] = 2;
        // stiction_positive[robot_->getJoint("LH_HFE")] = 2;
        // stiction_positive[robot_->getJoint("LH_KFE")] = 2;
        // stiction_positive[robot_->getJoint("RH_HAA")] = 2;
        // stiction_positive[robot_->getJoint("RH_HFE")] = 2;
        // stiction_positive[robot_->getJoint("RH_KFE")] = 2;
        // if(robot_name_.compare("crex")==0)
        // {
        //     stiction_positive[robot_->getJoint("LC_HAA")] = 2;
        //     stiction_positive[robot_->getJoint("LC_HFE")] = 2;
        //     stiction_positive[robot_->getJoint("LC_KFE")] = 2;
        //     stiction_positive[robot_->getJoint("RC_HAA")] = 2;
        //     stiction_positive[robot_->getJoint("RC_HFE")] = 2;
        //     stiction_positive[robot_->getJoint("RC_KFE")] = 2;
        // }

        // stiction_negative[robot_->getJoint("LF_HAA")] = 2;
        // stiction_negative[robot_->getJoint("LF_HFE")] = 2;
        // stiction_negative[robot_->getJoint("LF_KFE")] = 2;
        // stiction_negative[robot_->getJoint("RF_HAA")] = 2;
        // stiction_negative[robot_->getJoint("RF_HFE")] = 2;
        // stiction_negative[robot_->getJoint("RF_KFE")] = 2;
        // stiction_negative[robot_->getJoint("LH_HAA")] = 2;
        // stiction_negative[robot_->getJoint("LH_HFE")] = 2;
        // stiction_negative[robot_->getJoint("LH_KFE")] = 2;
        // stiction_negative[robot_->getJoint("RH_HAA")] = 2;
        // stiction_negative[robot_->getJoint("RH_HFE")] = 2;
        // stiction_negative[robot_->getJoint("RH_KFE")] = 2;
        // if(robot_name_.compare("crex")==0)
        // {
        //     stiction_negative[robot_->getJoint("LC_HAA")] = 2;
        //     stiction_negative[robot_->getJoint("LC_HFE")] = 2;
        //     stiction_negative[robot_->getJoint("LC_KFE")] = 2;
        //     stiction_negative[robot_->getJoint("RC_HAA")] = 2;
        //     stiction_negative[robot_->getJoint("RC_HFE")] = 2;
        //     stiction_negative[robot_->getJoint("RC_KFE")] = 2;
        // }

        // torque_offset[robot_->getJoint("LF_HAA")] = 0.0;
        // torque_offset[robot_->getJoint("LF_HFE")] = -2.0;
        // torque_offset[robot_->getJoint("LF_KFE")] = -4.0;
        // torque_offset[robot_->getJoint("RF_HAA")] = -1.0;
        // torque_offset[robot_->getJoint("RF_HFE")] = 0.0;
        // torque_offset[robot_->getJoint("RF_KFE")] = 0.0;
        // torque_offset[robot_->getJoint("LH_HAA")] = 0.0;
        // torque_offset[robot_->getJoint("LH_HFE")] = -2.0;
        // torque_offset[robot_->getJoint("LH_KFE")] = 0.0;
        // torque_offset[robot_->getJoint("RH_HAA")] = 0.0;
        // torque_offset[robot_->getJoint("RH_HFE")] = 0.0;
        // torque_offset[robot_->getJoint("RH_KFE")] = 0.0;
        // if(robot_name_.compare("crex")==0)
        // {
        //     torque_offset[robot_->getJoint("LC_HAA")] = 0.0;
        //     torque_offset[robot_->getJoint("LC_HFE")] = 0.0;
        //     torque_offset[robot_->getJoint("LC_KFE")] = 0.0;
        //     torque_offset[robot_->getJoint("RC_HAA")] = 0.0;
        //     torque_offset[robot_->getJoint("RC_HFE")] = 2.0;
        //     torque_offset[robot_->getJoint("RC_KFE")] = 0.0;
        // }

        auto robot_jacobian = this->makeFeetJacobian();
        this->updateLinearJacobian(q, robot_jacobian);

        for(auto leg: *this->getLegs())
        {
            int num_joints{0};
            Eigen::VectorXd tau_block = Eigen::VectorXd::Zero(leg->getNJoints());
            Eigen::VectorXd inv_dyn_tau_block = Eigen::VectorXd::Zero(leg->getNJoints());

            for(auto joint: *leg->getJoints())
            {
                tau_block(num_joints) = tau[joint];
                //remove friction torques
                //inv_dyn_tau[joint] -= torque_offset[joint] + sign_func(des_qd[joint])*stiction_positive
                                                             //+ (1-sign_func(des_qd[joint]))*stiction_negative
                inv_dyn_tau_block(num_joints) = inv_dyn_tau[joint];
                num_joints++;
            }
            estimated_feet_grf[leg] = robot_jacobian[leg].block<3,3>(0,0).transpose().inverse() * (inv_dyn_tau_block - tau_block);
            //extForces[leg] = jacobians_[leg].transpose().inverse() * (-tau_.segment(3*leg, 3));
        }
    }

    int RobotBase::computeNumStanceLegs(const LegDataMap<bool>& stance_legs) const
    {
        int leg_count{0};
        for(auto &leg_pair: stance_legs)
        {
            if (stance_legs[leg_pair.key_])
                leg_count++;
        }
        return leg_count;
    }

    void RobotBase::computeProprioHeight(const Eigen::Vector3d& w_rpy_b, const LegDataMap<bool>& stance_legs, const robotlib::LegDataMap<Eigen::Vector3d>& actual_foot_position, double& proprio_height) const
    {
        int num_stance_legs{computeNumStanceLegs(stance_legs)};
        if(num_stance_legs>0)
        {
            proprio_height = 0.0;
            //Compute foot position in horizontal frame
            Eigen::Matrix3d HF_R_b = utils::rpyToRot(Eigen::Vector3d(w_rpy_b[0], w_rpy_b[1], 0.0)).transpose();
            auto actual_foot_position_HF(actual_foot_position); //dynamic memory allocation is tacking place! - DMA
            for(auto &leg_pair : actual_foot_position_HF)
            {
                actual_foot_position_HF[leg_pair.key_] = HF_R_b*actual_foot_position[leg_pair.key_];
            }
            // Compute proprio height considering actual foot position in horizontal frame
            for(auto &leg_pair : stance_legs)
            {
                proprio_height += (-actual_foot_position_HF[leg_pair.key_](2) * (stance_legs[leg_pair.key_]))/num_stance_legs;
            }
        }
    }

        // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

    // Create a joint state
    robotlib::JointState RobotBase::makeJointState(const double value) const
    {
        robotlib::JointState joint_state(this->getLegs(), value);
        
        return joint_state;
    } // NRT

    // TODO
    Jacobian RobotBase::makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest) // NRT
    {
        fOrigin->getName();
        fDest->getName();

        std::cout << "makeJacobian function: TODO\n";
        return Jacobian(1);
    };

    // TODO: it should use makeJacobian
    Jacobian RobotBase::makeFootJacobian(const std::shared_ptr<Frame> frame) // NRT
    {
        // Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

        // const LimbBase *l = foot.getParentLimb();
        // const int nJoints = l->getNJoints();

        // return Jacobian(nJoints);
        frame->getName();

        std::cout << "makeFootJacobian-Input: foot function: TODO\n";
        return Jacobian(1);
    };

    // TODO: it should use makeJacobian
    Jacobian RobotBase::makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data) // NRT
    {
        return Jacobian(leg->getNJoints(), data);
    };

    LegDataMap<Jacobian> RobotBase::makeFeetJacobian(const double data) const // NRT
    {
        auto feetJac = this->makeLegDataMap<Jacobian>();

        for (auto &leg_pair : feetJac)
        {
            leg_pair.data_ = std::shared_ptr<Jacobian>(new Jacobian(leg_pair.key_->getNJoints(), data));
        }
        return feetJac;
    };

    void RobotBase::printRobotHierarchy()
    {
        for(auto leg: *this->getLegs())
        {
            std::cout << "\nLeg: " << leg->getName() << std::endl;

            for(auto joint : *leg->getJoints())
            {
                std::cout << leg->jointToParentName(joint) << " --> " << joint->getName() << " --> " << leg->jointToChildName(joint) << std::endl;
            }

            for(auto link : *leg->getLinks())
            {
                std::cout << leg->linkToParentName(link) << " --> " << link->getName() << " --> " << leg->linkToChildName(link) << std::endl;
            }
        }

        for(auto arm: *this->getArms())
        {
            std::cout << "\nArm: " << arm->getName() << std::endl;

            for(auto joint : *arm->getJoints())
            {
                std::cout << arm->jointToParentName(joint) << " --> " << joint->getName() << " --> " << arm->jointToChildName(joint) << std::endl;
            }

            for(auto link : *arm->getLinks())
            {
                std::cout << arm->linkToParentName(link) << " --> " << link->getName() << " --> " << arm->linkToChildName(link) << std::endl;
            }
        }
    }

} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_