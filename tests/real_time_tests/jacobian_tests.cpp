#include "robot_base.hpp"
#include "robot_factory.hpp"
#include "dynamic_memory_hooks.hpp"
#include <gtest/gtest.h>

TEST(RealTimeTest, get_functions)
{
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    
    // ** Define feet jacobian **
    robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> feet_jacobians {robot->makeFeetJacobian()};
    
    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    for(auto leg : *robot->getLegs())
    {
        Eigen::Map linear_jacobian {feet_jacobians[leg].getLinearJacobian()};
        Eigen::Map angular_jacobian {feet_jacobians[leg].getAngularJacobian()};
    }
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}