#include "robotlib/hyq/robot_limits.hpp"

namespace dls {
namespace dog {
namespace HyQ {


Limits::Limits()
{
    max_actuator_effort_ <<120.0, 3600.0, 3600.0, 120.0, 3600.0, 3600.0,120.0, 3600.0,3600.0, 120.0, 3600.0,3600.0; //N

}

Limits::~Limits(){}

void Limits::setMaxEffort(const dog::JointState & max_effort)
{
    max_actuator_effort_ = max_effort;
}

dog::JointState Limits::getTorqueLimits(const dls::dog::JointState & q)
{

    dog::JointState tau_max;
    tau_max.setZero();

    dls::dog::JointIdentifiers j_id;
    for (int i = 0; i<dls::dog::jointsCount; i++)
    {
        j_id = dls::dog::JointIdentifiers(i);
        double cylLength = dls::dog::HyQ::getCylinderLength( j_id, q(j_id) );
        double cylLever  = HyQ::getCylinderLeverArm( j_id, cylLength );
        if (cylLever == 0.0) {
            tau_max(j_id) = max_actuator_effort_(j_id);
        }else{
            tau_max(j_id) = max_actuator_effort_(j_id) * cylLever;}
    }
    return tau_max;
}

}
}
}
