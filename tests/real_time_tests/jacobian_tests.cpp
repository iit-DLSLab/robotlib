#include <gtest/gtest.h>
#include "robot_factory.hpp"
#include "dynamic_memory_hooks.hpp"

TEST(RealTimeTest, eigen_multiplication_dynamic_matrices)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>A,B,C;
    A.setOnes(500,500);
    B.setOnes(500,500);
    C.setOnes(500,500);

    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C = A*B;
    deactivate_hooks();
    is_dynamic_memory_used(true, false, false, true); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C.noalias() = A * B;
    deactivate_hooks();
    is_dynamic_memory_used(true, false, false, true); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    (C.noalias()=A.lazyProduct(B));
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
}