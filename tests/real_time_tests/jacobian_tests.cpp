#include "robot_base.hpp"
#include "dynamic_memory_hooks.hpp"
#include <gtest/gtest.h>

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

TEST(RealTimeTest, eigen_multiplication_fized_size_matrices)
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

TEST(RealTimeTest, eigen_operations_map_matrices)
{
    // ** Define data for the operations **
    const double fCoeff {0.5};
    const int n_rows {6};
    const int n_joints {18};
    Eigen::VectorXd q_home_filter_dynamic;
    Eigen::VectorXd q_home_dynamic;
    Eigen::VectorXd qd_dynamic;
    Eigen::VectorXd v_dynamic;
    Eigen::MatrixXd J_dynamic;
    Eigen::Matrix<double, n_joints, 1> q_home_filter_fixed_size;
    Eigen::Matrix<double, n_joints, 1> q_home_fixed_size;
    Eigen::Matrix<double, n_joints, 1> qd_fixed_size;
    Eigen::Matrix<double, n_rows, 1> v_fixed_size;
    Eigen::Matrix<double, n_rows, n_joints> J_fixed_size;

    q_home_filter_dynamic.setOnes(n_joints);
    q_home_dynamic.setOnes(n_joints);
    qd_dynamic.setOnes(n_joints);
    v_dynamic.setOnes(n_rows);
    J_dynamic.setOnes(n_rows, n_joints);   

    q_home_filter_fixed_size.setOnes();
    q_home_fixed_size.setOnes();
    qd_fixed_size.setOnes();
    v_fixed_size.setOnes();
    J_fixed_size.setOnes();

    // ** Operations with eigen structures **
    // Dynamic memory allocation/deallocation have not to take place (component-wise operation)
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    q_home_filter_dynamic = (1.0 - fCoeff) * q_home_filter_dynamic + fCoeff * q_home_dynamic;
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    q_home_filter_fixed_size = (1.0 - fCoeff) * q_home_filter_fixed_size + fCoeff * q_home_fixed_size;
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
    
    // Dynamic memory allocation/deallocation have to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    v_dynamic = J_dynamic * qd_dynamic;
    deactivate_hooks();
    is_dynamic_memory_used(true, false, false, true); // malloc, calloc, realloc, free

    // Dynamic memory allocation/deallocation have not to take place
    reset_variables_checking_use_of_dynamic_memory();
    activate_hooks();
    v_fixed_size = J_fixed_size * qd_fixed_size;
    deactivate_hooks();
    is_dynamic_memory_used(false, false, false, false); // malloc, calloc, realloc, free
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}