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

    void RobotBase::getMinJointAngle(const std::shared_ptr<Joint> joint, double &q_min) 
    { 
        q_min = joint->getMinAngle(); 
    };

    void RobotBase::getMaxJointAngle(const std::shared_ptr<Joint> joint, double &q_max) 
    { 
        q_max = joint->getMaxAngle(); 
    };

    void RobotBase::getMaxJointVelocity(const std::shared_ptr<Joint> joint, double &qd_max)
    { 
        qd_max = joint->getMaxVelocity(); 
    };

    void RobotBase::getMaxJointEffort(const std::shared_ptr<Joint> joint, double &tau_max) 
    { 
        tau_max = joint->getMaxEffort(); 
    };

    int RobotBase::computeNumStanceLegs(const LegDataMap<bool>& stance_legs) const
    {
        int leg_count{0};
        for(auto leg_pair: stance_legs)
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
            for(auto leg_pair : actual_foot_position_HF)
            {
                actual_foot_position_HF[leg_pair.key_] = HF_R_b*actual_foot_position[leg_pair.key_];
            }
            // Compute proprio height considering actual foot position in horizontal frame
            for(auto leg_pair : stance_legs)
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
        std::cout << "makeFootJacobian-Input: foot function: TODO\n";
        return Jacobian(1);
    };

    // TODO: it should use makeJacobian
    Jacobian RobotBase::makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data ) // NRT
    {
        return Jacobian(leg->getNJoints(), data);
    };

    LegDataMap<Jacobian> RobotBase::makeFeetJacobian(const double data) const // NRT
    {
        auto feetJac = this->makeLegDataMap<Jacobian>();

        for (auto &leg_pair : feetJac)
        {
            leg_pair.data_ = std::shared_ptr<Jacobian>(new Jacobian(leg_pair.key_->getNJoints()));
        }
        return feetJac;
    };
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_