#ifndef _ROBOTLIB_ROBOT_BASE_CPP_
#define _ROBOTLIB_ROBOT_BASE_CPP_

#include "robot_base.hpp"

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

    int RobotBase::compute_stance_legs(const robotlib::LegDataMap<bool>& stance_legs)
    {
        int leg_count{0};
        for(auto leg_pair: stance_legs)
        {
            if (stance_legs[leg_pair.key_])
                leg_count++;
        }
        return leg_count;
    }

    double RobotBase::computeProprioHeight(const Eigen::Vector3d w_rpy_b, const robotlib::LegDataMap<bool>& stance_legs, const robotlib::LegDataMap<Eigen::Vector3d>& actual_foot_position, const double actual_proprio_height)
    {
        double proprio_height = actual_proprio_height; 
        int n_stance_legs{compute_stance_legs(stance_legs)};
        if(n_stance_legs>0)
        {
            proprio_height = 0.0;

            //Compute foot position in horizontal frame
            auto actual_foot_pos_HF = this->pRobot->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
            auto HF_R_b = dls::math::rpyToRot(Eigen::Vector3d(w_rpy_b[0], w_rpy_b[1], 0.0)).transpose();
            for(auto leg_pair : actual_foot_pos_HF)
            {
                actual_foot_pos_HF[leg_pair.key_] = HF_R_b*actual_foot_position[leg_pair.key_];
            }
            // Compute proprio height
            for(auto leg_pair : stance_legs)
            {
                proprio_height += (-actual_foot_pos_HF[leg_pair.key_](2) * (stance_legs[leg_pair.key_]))/n_stance_legs;
            }
        }
        return proprio_height;
    }


} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_