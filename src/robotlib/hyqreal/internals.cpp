#include "robotlib/hyqreal/internals.hpp"
#include "robotlib/hyqreal/mechanical_constants.h"
#include "robotlib/base/leg_data_map.hpp"

using namespace dls::dog;

const HyQReal::internal::MagicNumbers HyQReal::internal::misc_cfg;

HyQReal::internal::MagicNumbers::MagicNumbers()
{
    HFEJointToMotorOffset[LF] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[RF] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[LH] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[RH] =  LF_HFE_JOINT_MOTOR_OFF;

    HAAJointToMotorOffset[LF] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[RF] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[LH] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[RH] = LF_HAA_JOINT_MOTOR_OFF;

}
