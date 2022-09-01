
#ifndef _ROBOTLIB_DATA_MAP_CPP_
#define _ROBOTLIB_DATA_MAP_CPP_

#include "data_map.hpp"
#include "robot_base.hpp"
#include "utils.hpp"

namespace robotlib
{
    int DataHelper::getRobotNumLinks(std::shared_ptr<RobotBase> pRobot)
    {
        return pRobot->getNLINKS();
    }

    int DataHelper::getRobotNumJoints(std::shared_ptr<RobotBase> pRobot)
    {
        return pRobot->getNJOINTS();
    }

    const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> DataHelper::getLegs(std::shared_ptr<RobotBase> pRobot)
    {
        return pRobot->getLegs();
    }
}

#endif