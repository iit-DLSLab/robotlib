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
    
    double RobotBase::getMinJointAngle(const Joint* joint) { return joint->getMinAngle(); };

    double RobotBase::getMaxJointAngle(const Joint* joint) { return joint->getMaxAngle(); };

    double RobotBase::getMaxJointVelocity(const Joint* joint) { return joint->getMaxVelocity(); };

    double RobotBase::getMaxJointEffort(const Joint* joint) { return joint->getMaxEffort(); };

    robotlib::LimbDataMap<Eigen::Vector3d> RobotBase::estimateLimbsGRF(const Eigen::Matrix<double,7,1>& pose, const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau)
    {
        robotlib::JointState inv_dyn_tau = this->makeJointState();	// remove this NRT instatiation!
        this->inverseDynamics(pose,
								Eigen::Matrix<double, 6, 1>::Zero(),
                                Eigen::Matrix<double, 6, 1>::Zero(),
                                q,
                                qd,
                                qdd,
                                {},
                                inv_dyn_tau);

        auto estimated_feet_grf = this->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

        int limb_id = 0;
        for(auto &limb: this->getLimbs())
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

            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, this->getNJOINTS());
            const std::string foot_name = utils::toLower(limb.getName())+"_foot";
            this->computeLimbsJacobian(q, foot_name, J);            
            const int njoints_limb = limb.getNJoints();
            Eigen::MatrixXd J_limb = J.block(0,limb_id,3,njoints_limb);
            limb_id+=njoints_limb;

            estimated_feet_grf[limb] = J_limb.transpose().inverse() * (inv_dyn_tau_block - tau_block);
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
        std::vector<Jacobian> container;

        int j{0};
		for(auto& limb : this->getLimbs())
		{
            container.push_back(Jacobian(limb.getNJoints(), data));
        }

        return this->makeLimbDataMap<Jacobian>(container);
    };

    Jacobian RobotBase::makeFootJacobian(const LimbBase& leg, const double data) // NRT
    {
        return Jacobian(leg.getNJoints(), data);
    };

    Eigen::Vector3d RobotBase::computeCoMFromBase(const robotlib::JointState &q,
                                const Eigen::Matrix<double, 7, 1> &robot_pose)
    {        
        Eigen::Matrix3d w_R_b = robotlib::utils::quatToRotMat(Eigen::Quaterniond(robot_pose.block<4,1>(3,0))).transpose(); // orientation of the world frame expressed in base frame
        Eigen::Vector3d offCoM = computeWholeBodyCoM(q);
        return robot_pose.block<3,1>(0,0) + w_R_b * offCoM;         //CoM is in the world frame, off CoM is in base frame
    }

    Eigen::Vector3d RobotBase::computeBaseFromCoM( const JointState &q,
                                    const Eigen::Matrix<double, 4, 1> &base_orient,
                                    const Eigen::Vector3d &CoM){
        Eigen::Matrix3d w_R_b = robotlib::utils::quatToRotMat(Eigen::Quaterniond(base_orient)).transpose(); // orientation of the world frame expressed in base frame

        Eigen::Vector3d offCoM = computeWholeBodyCoM(q);
        return CoM - w_R_b * offCoM;          //CoM is in the world frame, off CoM is in base frame
    }

} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_