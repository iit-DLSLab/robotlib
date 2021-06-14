#include <gtest/gtest.h>
#include <iostream>

TEST(CustomTests, numbers)
{
    std::cout << "TEST ON NUMBERS" << std::endl;
    int n1{10}, n2{20};
    ASSERT_FALSE(n1 > n2);
    ASSERT_TRUE(n1 < n2);
}

TEST(CustomTests, strings)
{
    std::cout << "TEST ON STRINGS" << std::endl;
    std::string s1{"Dog"}; std::string s2{"Dog"}; std::string s3{"Robot"};
    ASSERT_EQ(s1.compare(s2), 0);
    ASSERT_FALSE(s1.compare(s3) == 0);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}