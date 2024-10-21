cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(ament_cmake_pal QUIET)

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/joystick.yaml
    module/play_motion2.yaml
    module/twist_mux.yaml
  )
  pal_register_modules(module/20_localization.yaml
                      module/20_localization_w_composition.yaml
                      module/20_navigation.yaml
                      module/20_navigation_w_composition.yaml
                      module/20_slam.yaml)
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_export_include_directories("~/stero/src/STERO_24z_AntoniKowalski_TomaszZalewski/hello_moveit/")

install(
  PROGRAMS scripts/regen_em_file.py
  DESTINATION lib/${PROJECT_NAME}
)

#ament_auto_package(INSTALL_TO_SHARE config launch)

#ament_export_include_directories("~/stero/src/STERO_24z_AntoniKowalski_TomaszZalewski/hello_moveit/include")

ament_auto_package(INSTALL_TO_SHARE config launch params)

#ament_export_include_directories("~/stero/src/STERO_24z_AntoniKowalski_TomaszZalewski/hello_moveit/include")
#ament_auto_package(INSTALL_TO_SHARE config launch models worlds urdf meshes)
