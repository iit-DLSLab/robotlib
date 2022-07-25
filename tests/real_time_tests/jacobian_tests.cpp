#include <gtest/gtest.h>
#include "robot_factory.hpp"
#include "dynamic_memory_hooks.hpp"

TEST(RealTimeTest, constructor)
{
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    
    // ** Define feet jacobian **    
    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    robotlib::RobotBase::Jacobian foot_jacobian {robot->makeFootJacobian(robot->getLink("LF_LOWERLEG"))};
    deactivate_hooks();
    is_dynamic_memory_used(true, false, false, false); // malloc, calloc, realloc, free
}

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

TEST(RealTimeTest, print_function)
{
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    
    // ** Define feet jacobian **
    robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> feet_jacobians {robot->makeFeetJacobian()};
    
    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    feet_jacobians.print();
    is_dynamic_memory_used(true, false, false, true); // malloc, calloc, realloc, free
}

TEST(RealTimeTest, equal_operator)
{
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    
    // ** Define feet jacobian **
    robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> feet_jacobians {robot->makeFeetJacobian()};
    robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> feet_jacobians_2 {robot->makeFeetJacobian()};
    
    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    feet_jacobians = feet_jacobians_2;
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
}