#include "robotlib/hyq/internals.hpp"
#include "robotlib/hyq/constants.hpp"

const dls::dog::HyQ::internal::MagicNumbers dls::dog::HyQ::internal::misc_cfg;

dls::dog::HyQ::internal::MagicNumbers::MagicNumbers()
{
    HAAJointToMotorOffset[dog::LF] = dls::dog::HyQ::LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[dog::RF] = dls::dog::HyQ::LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[dog::LH] = dls::dog::HyQ::LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[dog::RH] = dls::dog::HyQ::LF_HAA_JOINT_MOTOR_OFF;
}
