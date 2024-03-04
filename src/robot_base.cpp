#ifndef _ROBOTLIB_ROBOT_BASE_CPP_
#define _ROBOTLIB_ROBOT_BASE_CPP_

#include "robot_base.hpp"
#include "utils/utils.hpp"
#include "utils/container.hpp"

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

    robotlib::LimbDataMap<Eigen::Vector3d> RobotBase::estimateFeetGRF(const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau, const Eigen::Matrix<double, 6,1>& g_b)
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


        auto estimated_feet_grf = this->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        auto robot_jacobian = this->makeFeetJacobian();
        this->updateLinearJacobian(q, robot_jacobian);

        for(auto& limb : this->getLimbs())
        {
            int num_joints{0};
            Eigen::VectorXd tau_block = Eigen::VectorXd::Zero(limb.getNJoints());
            Eigen::VectorXd inv_dyn_tau_block = Eigen::VectorXd::Zero(limb.getNJoints());

            for(auto& joint: limb.getJoints())
            {
                tau_block(num_joints) = tau[joint];
                //remove friction torques
                //inv_dyn_tau[joint] -= torque_offset[joint] + sign_func(des_qd[joint])*stiction_positive
                                                             //+ (1-sign_func(des_qd[joint]))*stiction_negative
                inv_dyn_tau_block(num_joints) = inv_dyn_tau[joint];
                num_joints++;
            }
            estimated_feet_grf[limb] = robot_jacobian[limb].block<3,3>(0,0).transpose().inverse() * (inv_dyn_tau_block - tau_block);
            //extForces[limb] = jacobians_[limb].transpose().inverse() * (-tau_.segment(3*limb, 3));
        }
        return estimated_feet_grf;
    }

    int RobotBase::computeNumStanceLegs(const LimbDataMap<bool>& stance_legs) const
    {
        int limb_count{0};

        for(auto& limb_pair: stance_legs)
        {
            if (stance_legs[limb_pair])
                limb_count++;
        }
        return limb_count;
    }

    void RobotBase::computeProprioHeight(const Eigen::Vector3d& w_rpy_b, const LimbDataMap<bool>& stance_legs, const robotlib::LimbDataMap<Eigen::Vector3d>& actual_foot_position, double& proprio_height) const
    {
        int num_stance_legs{computeNumStanceLegs(stance_legs)};
        if(num_stance_legs>0)
        {
            proprio_height = 0.0;
            //Compute foot position in horizontal frame
            Eigen::Matrix3d HF_R_b = utils::rpyToRot(Eigen::Vector3d(w_rpy_b[0], w_rpy_b[1], 0.0)).transpose();
            auto actual_foot_position_HF(actual_foot_position); //dynamic memory allocation is tacking place! - DMA

            for(auto& limb_pair : actual_foot_position_HF)
            {
                actual_foot_position_HF[limb_pair] = HF_R_b*actual_foot_position[limb_pair];
            }

            // Compute proprio height considering actual foot position in horizontal frame
            for(auto& limb_pair : stance_legs)
            {
                proprio_height += (-actual_foot_position_HF[limb_pair.getKey()](2) * (stance_legs[limb_pair]))/num_stance_legs;
            }
        }
    }

    JointState RobotBase::makeJointState(const double& value) const
    {
        JointState joint_state(this->getLimbs(), value);
        
        return joint_state;
    }

    LimbDataMap<Jacobian> RobotBase::makeFeetJacobian(const double& data) const
    {
        return this->makeLimbDataMap<Jacobian>(
            [&]() -> std::vector<Jacobian>
            {
                std::vector<Jacobian> container;

				for(auto& limb : this->getLimbs())
				{
                    container.push_back(Jacobian(limb.getNJoints(), data));
                }
                return container;
            }()
        );
    };

    // void RobotBase::printRobotHierarchy()
    // {
    //     for(auto& limb: this->getLimbs())
    //     {
    //         std::cout << "\nlimb: " << limb.getName() << std::endl;

    //         for(auto& joint : limb.getJoints())
    //         {
    //             std::cout << limb.jointToParentName(joint) << " --> " << joint.getName() << " --> " << limb.jointToChildName(joint) << std::endl;
    //         }

    //         for(auto& link : limb.getLinks())
    //         {
    //             std::cout << limb.linkToParentName(link) << " --> " << link.getName() << " --> " << limb.linkToChildName(link) << std::endl;
    //         }
    //     }
    // }

} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_