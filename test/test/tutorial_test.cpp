#include <gtest/gtest.h>
#include <compass_conversions/topic_names.h>

TEST(package_name, a_first_test)
{
  ASSERT_EQ(4, 2 + 2);
  
  compass_conversions::parseAzimuthTopicName("asd");
  ASSERT_EQ(4, 2 + 3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  return RUN_ALL_TESTS();
}