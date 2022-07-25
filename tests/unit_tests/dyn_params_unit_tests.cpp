/**
 * @file dyn_params_unit_tests.cpp
 * @brief Unit tests for DynParams class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "dyn_params.hpp"

/**
 * @brief Set of unit tests for DynParams::getCoM function
 */
TEST(DynParamsUnitTests, getCoM)
{
  const double mass{1.0};
  const Eigen::Matrix3d inertia(
      (Eigen::Matrix3d() << 1.0, 1.0, 1.0,
       1.0, 1.0, 1.0,
       1.0, 1.0, 1.0)
          .finished()); /// Way to initialize a const Eigen matrix
  /**
      * @test CoM vector with all 0.0 values
      */
  {
    const Eigen::Vector3d com{0.0, 0.0, 0.0};
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getCoM(), Eigen::Vector3d(0.0, 0.0, 0.0));
  }
  /**
      * @test CoM vector with all 1.0 values
      */
  {
    const Eigen::Vector3d com{1.0, 1.0, 1.0};
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getCoM(), Eigen::Vector3d(1.0, 1.0, 1.0));
  }
  /**
      * @test CoM vector with empty vector
      */
  {
    const Eigen::Vector3d com{};
    robotlib::DynParams dyn_params{com, mass, inertia};

    /// TODO: If empty vector it is filled with 1.0 values. Correct this behaviour
    ASSERT_EQ(dyn_params.getCoM(), Eigen::Vector3d(1.0, 1.0, 1.0));
  }
}

/**
 * @brief Set of unit tests for DynParams::getMass function
 */
TEST(DynParamsUnitTests, getMass)
{
  const Eigen::Vector3d com{1.0, 1.0, 1.0};
  const Eigen::Matrix3d inertia(
      (Eigen::Matrix3d() << 1.0, 1.0, 1.0,
       1.0, 1.0, 1.0,
       1.0, 1.0, 1.0)
          .finished());
  /**
      * @test Mass with value 0.0
      */
  {
    const double mass{0.0};
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getMass(), 0.0);
  }
  /**
      * @test Mass with value 1.0
      */
  {
    const double mass{1.0};
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getMass(), 1.0);
  }
  /**
      * @test Mass with empty value
      */
  {
    const double mass{};
    robotlib::DynParams dyn_params{com, mass, inertia};

    /// TODO: If empty variable it is initialized to 0.0. Correct this behaviour
    ASSERT_EQ(dyn_params.getMass(), 0.0);
  }
}

/**
 * @brief Set of unit tests for DynParams::getInertia function
 */
TEST(DynParamsUnitTests, getInertia)
{
  const Eigen::Vector3d com{1.0, 1.0, 1.0};
  const double mass{1.0};
  /**
      * @test Inertia with all 0.0 values
      */
  {
    const Eigen::Matrix3d inertia(
        (Eigen::Matrix3d() << 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0)
            .finished());
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getInertia(), Eigen::Matrix3d().setZero());
  }
  /**
      * @test Inertia with all 1.0 values
      */
  {
    const Eigen::Matrix3d inertia(
        (Eigen::Matrix3d() << 1.0, 1.0, 1.0,
         1.0, 1.0, 1.0,
         1.0, 1.0, 1.0)
            .finished());
    robotlib::DynParams dyn_params{com, mass, inertia};

    ASSERT_EQ(dyn_params.getInertia(), Eigen::Matrix3d().setOnes());
  }
  /**
      * @test Inertia with empty matrix
      */
  {
    const Eigen::Matrix3d inertia{};
    robotlib::DynParams dyn_params{com, mass, inertia};

    /// TODO: If empty vector it is filled with 1.0 values. Correct this behaviour
    ASSERT_EQ(dyn_params.getInertia(), Eigen::Matrix3d().setOnes());
  }
}