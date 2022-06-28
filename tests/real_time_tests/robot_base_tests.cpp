#include "robot_base.hpp"
#include "robot_factory.hpp"
#include "dynamic_memory_hooks.hpp"
#include <gtest/gtest.h>


// In order to do the tests you need in install the dummy robots.
// To do so, just do make install inside the build folder of robotlib, from docker root terminal. 

TEST(RobotBaseRealTimeTests, create_robot)
{
    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    deactivate_hooks();
    is_dynamic_memory_used(true, true, false, true); // malloc, calloc, realloc, free 
    std::cout << "free seems to be called few times w.r.t. malloc one. TODO: understand why.\nTo verify: uncomment prints in malloc and free hooks.\n";
  
}

TEST(RobotBaseRealTimeTests, getLegs)
{
    // ** Define robot **
    std::shared_ptr<robotlib::RobotBase> robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    for (auto leg : *robot->getLegs())
    {
        // do nothing
    }
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free   
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
