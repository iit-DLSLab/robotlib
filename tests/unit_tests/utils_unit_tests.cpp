/**
 * @file utils_unit_tests.cpp
 * 
 * @brief Unit tests for ContainerBase and Container classes
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "utils/container.hpp"

TEST(UtilsUnitTests, Containers)
{
    std::cout << "Testing correct storing for non share pointer data...\n";

    const int N {10};
    std::array<double, N> data{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    robotlib::Container<double, N> container1(data);

    for (auto i{0}; i < N-1; i++)
    {
        std::cout << "test  " << container1[i] << std::endl;
        EXPECT_EQ(container1[i], i);
    }

    std::cout << "Testing correct storing for share pointer data... \n";
    std::array<std::shared_ptr<double>, N> data_shp;

    for (auto i{0}; i < N; i++)
    {
        data_shp[i] = std::make_shared<double>(i);
    }

    robotlib::Container<double, N> container2(data_shp);

    for (auto i{0}; i < N; i++)
    {
        EXPECT_EQ(container2[i], i);
    }
}