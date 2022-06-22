// #define EIGEN_RUNTIME_NO_MALLOC

#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>
#include "dynamic_memory_hooks.hpp"

TEST(RealTimeTest, eigen_multiplication)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>A,B,C;
    A.setOnes(500,500);
    B.setOnes(500,500);
    C.setOnes(500,500);

    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
   
    // Eigen::internal::set_is_malloc_allowed(false);
    // C = A*B;
    // C.noalias() = A * B;
    (C.noalias()=A.lazyProduct(B));
    // Eigen::internal::set_is_malloc_allowed(true);
    
    is_dynamic_memory_used(false, false, false, false);
    
    deactivate_hooks();

      
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}