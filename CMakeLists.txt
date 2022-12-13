

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(planar_segmentation)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "") # works


find_package(PCL 1.2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Boost REQUIRED )
find_package(LEMON REQUIRED)




#add_executable (example main.cpp)
#target_link_libraries (example Eigen3::Eigen)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(visualizer_cloud_and_path src/visualizer_cloud_and_path.cpp)
target_link_libraries(visualizer_cloud_and_path PUBLIC ${PCL_LIBRARIES}
  Auxilary)
target_include_directories(visualizer_cloud_and_path PUBLIC include)

add_library(Auxilary src/Auxilary.cpp)
target_include_directories(Auxilary PUBLIC include ${LEMON_INCLUDE_DIR})
target_link_libraries(Auxilary PUBLIC ${PCL_LIBRARIES}
                                      ${LEMON_LIBRARIES}
                                      visualizer_cloud_and_path)




add_executable (visualize main.cpp)
target_link_libraries (visualize PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(visualize PRIVATE ${LEMON_LIBRARIES})
target_link_libraries(visualize PRIVATE Auxilary visualizer_cloud_and_path)
target_include_directories(visualize PRIVATE include ${LEMON_INCLUDE_DIR})


add_executable(rebuildRRTTree src/rebuildRRTTree.cpp)
target_link_libraries (rebuildRRTTree PRIVATE ${LEMON_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen
  Auxilary
  )
target_include_directories( rebuildRRTTree PRIVATE include ${LEMON_INCLUDE_DIR})
