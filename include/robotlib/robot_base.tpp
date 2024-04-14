#ifndef _ROBOTLIB_ROBOT_BASE_TPP_
#define _ROBOTLIB_ROBOT_BASE_TPP_

#include "robot_base.hpp"

namespace robotlib
{
    // Create a llimb data map pair
    template <class Data>
    LimbDataMap<Data> RobotBase::makeLimbDataMap(const Data& data) const
    { 
        return LimbDataMap<Data>(this->getLimbs(), data); 
    }

    template <class Data>
    LimbDataMap<Data> RobotBase::makeLimbDataMap(const std::vector<Data>& data) const
    {
        return LimbDataMap<Data>(this->getLimbs(), data); 
    }

    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap(const Data& data) const
    { 
        return LinkDataMap<Data>(this->getLinks(), data); 
    }

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const std::vector<Data>& data) const
    {
        return JointDataMap<Data>(this->getJoints(), data); 
    }

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const Data& data) const
    { 
        return JointDataMap<Data>(this->getJoints(), data); 
    }

    // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

    // Create a joint state
    // // TODO
	// template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    // Jacobian Robot::makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest) // NRT
    // {
    //     fOrigin->getName();
    //     fDest->getName();

    //     std::cout << "makeJacobian function: TODO\n";
    //     return Jacobian(1);
    // };

    // // TODO: it should use makeJacobian
	// template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    // Jacobian Robot::makeFootJacobian(const std::shared_ptr<Frame> frame) // NRT
    // {
    //     // Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

    //     // const Limb *l = foot.getParentLimb();
    //     // const int nJoints = l->getNJoints();

    //     // return Jacobian(nJoints);
    //     frame->getName();

    //     std::cout << "makeFootJacobian-Input: foot function: TODO\n";
    //     return Jacobian(1);
    // };

    // // TODO: it should use makeJacobian
	// template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    // Jacobian Robot::makeFootJacobian(const std::shared_ptr<Limb> limb, const double data) // NRT
    // {
    //     return Jacobian(limb->getNJoints(), data);
    // };

} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_TPP_