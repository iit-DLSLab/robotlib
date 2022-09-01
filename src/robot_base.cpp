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

        for (auto leg : *(this->getLegs()))
        {
            feetJac[leg].init(leg->getNJoints(), data);
        }
        return feetJac;
    };

} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_CPP_