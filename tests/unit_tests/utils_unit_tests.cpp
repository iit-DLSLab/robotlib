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
    const int N {10};

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