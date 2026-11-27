cmake_minimum_required(VERSION 3.5)
project(pilsbot_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No license is chosen for this package yet (package.xml), so there is no
  # copyright notice to check. Remove this line once there is one.
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_log_kohl test/test_log_kohl.py)
endif()

install(PROGRAMS scripts/static_data_republisher DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")
install(DIRECTORY config foxglove DESTINATION share/${PROJECT_NAME})

ament_package()
