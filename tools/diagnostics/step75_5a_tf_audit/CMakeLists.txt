cmake_minimum_required(VERSION 3.16)
project(sie_step75_5a_tf_audit LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(sie_tf_receipts tf_receipts.cpp)
target_compile_features(sie_tf_receipts PRIVATE cxx_std_17)
target_link_libraries(sie_tf_receipts PRIVATE rclcpp::rclcpp tf2_msgs::tf2_msgs__rosidl_typesupport_cpp)
