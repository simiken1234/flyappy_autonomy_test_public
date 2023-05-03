find_package(GTest REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

include(GoogleTest)

add_executable(test_my_feature test_my_feature.cpp)
target_link_libraries(test_my_feature
  flyappy
  flyappy_ros
  gtest
  gtest_main
)
gtest_discover_tests(test_my_feature)
