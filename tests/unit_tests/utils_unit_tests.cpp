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
#include "utils.hpp"

TEST(UtilsUnitTests, Containers)
{
    std::cout << "Testing correct storing for non share pointer data...\n";

    const int N {10};
    double value {1};
    std::array<double, N> data;

    for (auto &d : data)
    {
        d = value;
    }
    robotlib::Container<double, N> container(data);

    for (auto d : container)
    {
        EXPECT_EQ(d, value);
    }

    std::cout << "Testing correct storing for share pointer data... \n";
    value = 2;
    std::array<std::shared_ptr<double>, N> data_shp;

    for (auto &d : data_shp)
    {
        d = std::make_shared<double>(value);
    }

    robotlib::Container<std::shared_ptr<double>, N> container_shp(data_shp);

    for (auto d : container_shp)
    {
        EXPECT_EQ(*d, value);
    }
}