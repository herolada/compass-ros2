// #include <magnetometer_pipeline/bias_remover.h>
// #include <compass_conversions/topic_names.h>
#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <rclcpp/utilities.hpp>
// #include <imu_transformer/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>

// const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName("asd");


// double det(const std::array<double, 9>& mat)
// {
//   return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(mat.data()).determinant();
// }

// const std::chrono::nanoseconds dur(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
// rclcpp::sleep_for(dur);
