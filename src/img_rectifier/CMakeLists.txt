cmake_minimum_required(VERSION 3.8)
project(img_rectifier)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake_auto REQUIRED)
#set(OpenCV_DIR "~/libraries/libopencv-4.5.5/lib/cmake/opencv4/")
#find_package(OpenCV 4.5 REQUIRED)


#set(OpenCV_DIR "/home/zeys/libraries/libopencv-4.5.5/share/opencv4/")
# find dependencies
find_package(OpenCV  REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(image_rectifier_node SHARED
        include/img_rectifier/image_rectifier.hpp
        src/image_rectifier.cpp
        )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_rectifier_node
        ${OpenCV_LIBRARIES}
        )

# works as main  plugın takes string be carefull about write string correctly
rclcpp_components_register_node(image_rectifier_node
        PLUGIN "ImageCorrector::ImageRectifierNode"
        EXECUTABLE image_rectifier
        )

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        params
)
