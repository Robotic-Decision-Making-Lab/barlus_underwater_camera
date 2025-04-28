cmake_minimum_required(VERSION 3.23)
project(barlus_image_pipeline)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

generate_parameter_library(image_proxy_parameters src/image_proxy_parameters.yaml)
generate_parameter_library(track_markers_parameters src/track_markers_parameters.yaml)

# image_proxy_node library
add_library(image_proxy_component SHARED)
target_sources(
    image_proxy_component
    PRIVATE src/image_proxy.cpp src/common.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/barlus_image_pipeline/image_proxy.hpp
)
target_compile_features(image_proxy_component PUBLIC cxx_std_23)
target_link_libraries(image_proxy_component
  PRIVATE image_proxy_parameters
  PUBLIC
    PkgConfig::gstreamer
    PkgConfig::gstreamer-app
    rclcpp::rclcpp
    rclcpp_components::component
    cv_bridge::cv_bridge
    camera_info_manager::camera_info_manager
    image_transport::image_transport
    ${geometry_msgs_TARGETS}
    ${OpenCV_LIBS}
)

rclcpp_components_register_node(image_proxy_component
    PLUGIN "barlus::ImageProxyNode"
    EXECUTABLE image_proxy_node
)

# track_markers_node library
add_library(track_markers_component SHARED)
target_sources(
    track_markers_component
    PRIVATE src/track_markers.cpp src/common.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/barlus_image_pipeline/track_markers.hpp
)
target_compile_features(track_markers_component PUBLIC cxx_std_23)
target_link_libraries(track_markers_component
  PRIVATE track_markers_parameters
  PUBLIC
    PkgConfig::gstreamer
    PkgConfig::gstreamer-app
    rclcpp::rclcpp
    rclcpp_components::component
    cv_bridge::cv_bridge
    camera_info_manager::camera_info_manager
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2::tf2
    ${geometry_msgs_TARGETS}
    ${OpenCV_LIBS}
)

rclcpp_components_register_node(track_markers_component
    PLUGIN "barlus::TrackMarkersNode"
    EXECUTABLE track_markers_node
)

install(
    TARGETS
        image_proxy_component
        track_markers_component
        image_proxy_parameters
        track_markers_parameters
    EXPORT export_barlus_image_pipeline
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

install(DIRECTORY launch config DESTINATION share/barlus_image_pipeline)

ament_export_targets(export_barlus_image_pipeline HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "rclcpp"
  "geometry_msgs"
  "rclcpp_components"
  "cv_bridge"
  "camera_info_manager"
  "image_transport"
  "tf2_geometry_msgs"
  "tf2"
)

ament_package()
