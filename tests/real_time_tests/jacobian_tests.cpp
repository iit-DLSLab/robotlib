#include "robot_base.hpp"
#include <gtest/gtest.h>
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

TEST(RealTimeTest, eigen_multiplication_static_matrices)
{
    Eigen::Matrix<double, 120, 120>A,B,C;
    A.setOnes();
    B.setOnes();
    C.setOnes();

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C = A*B;
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C.noalias() = A * B;
    deactivate_hooks(); 
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free 

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    (C.noalias()=A.lazyProduct(B));
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
}

TEST(RealTimeTest, eigen_multiplication_map_matrices)
{
    // ** Define vectors to be mapped **
    const int n_rows {25};
    const int n_cols {25};
    const int n_data {n_rows*n_cols};
    double* data_A = new double[n_data];
    double* data_B = new double[n_data];
    double* data_C = new double[n_data];

    for (int i=0; i< n_data; i++)
    {
        data_A[i] = 1.0;
        data_B[i] = 1.0;
        data_C[i] = 1.0;
    }

    // ** Define eigen map structures **
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A {data_A, n_rows, n_cols};
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> B {data_B, n_rows, n_cols};
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> C {data_C, n_rows, n_cols};

    // ** Multiply eigen map structures **
    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C = A*B;
    deactivate_hooks();
    is_dynamic_memory_used(true, false, false, true); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    C.noalias() = A * B;
    deactivate_hooks(); 
    is_dynamic_memory_used(false, false, false, true); // malloc, calloc, realloc, free 

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    (C.noalias()=A.lazyProduct(B));
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
    
}