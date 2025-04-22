#include <gtest/gtest.h>
// #include <compass_conversions/topic_names.h>
#include <Eigen/Core>
// #include <Eigen/Dense>
#include <Eigen/LU> 

TEST(package_name, a_first_test)
{
  ASSERT_EQ(4, 2 + 2);
  
  // compass_conversions::parseAzimuthTopicName("asd");
  ASSERT_EQ(4, 2 + 3);
}

auto det(const std::array<double, 9>& mat)
{
  // return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(mat.data()).determinant();
  Eigen::Matrix<double, 3, 3> m(mat.data());
  return m.determinant();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  return RUN_ALL_TESTS();
}