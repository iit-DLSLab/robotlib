#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

void setMap(Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> map)
{
    float r = 3;
    float c = 4;
    int count = 0;
    for (int i = 0; i < r; ++i)
    {
        for (int j = 0; j < c; ++j)
        {
            map(i, j) = count;
            count++;
        }
    }
}

TEST(EigenMapTest, setMap)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto name_dq = dummy_quadruped->getName();

    /// Ground truth
    std::string name_gt{"Quadruped"};

    /// Assert conditions
    ASSERT_EQ(name_dq, name_gt);
    ASSERT_EQ(typeid(name_dq).name(), typeid(name_gt).name());

    const int n_dims = 5;

    int r = 3;
    int c = 4;

    double *array = new double[9];
    for (int i = 0; i < 9; i++)
    {
        array[i] = i;
    }
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> map(&array[0], 3, 3);
    std::cout << map << std::endl;

    // Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> map2(NULL, 3, 3);
    // new (&map2) Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(array, 3, 3);
    // std::cout << map2 << std::endl;
    std::cout << "************" << std::endl;
    Eigen::Matrix<double, 2, 2> temp{map.block(0, 0, 2, 2)};

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> mapResized(&temp(0, 0), 2, 2);

    std::cout << mapResized << std::endl;
}