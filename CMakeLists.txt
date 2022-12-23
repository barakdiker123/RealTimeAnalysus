

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(planar_segmentation)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "") # works


find_package(PCL 1.2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Boost REQUIRED )
find_package(LEMON REQUIRED)

if(UNIX AND NOT APPLE)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
endif()



#add_executable (example main.cpp)
#target_link_libraries (example Eigen3::Eigen)


#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(visualizer_cloud_and_path src/visualizer_cloud_and_path.cpp)
target_link_libraries(visualizer_cloud_and_path PUBLIC ${PCL_LIBRARIES}
  Auxilary)
target_include_directories(visualizer_cloud_and_path PUBLIC include)
target_include_directories(visualizer_cloud_and_path PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(visualizer_cloud_and_path PRIVATE ${PCL_LIBRARY_DIRS})
#target_add_definitions(visualizer_cloud_and_path PRIVATE ${PCL_DEFINITIONS})

add_library(Auxilary src/Auxilary.cpp)
target_include_directories(Auxilary PUBLIC include ${LEMON_INCLUDE_DIR})
target_link_libraries(Auxilary PUBLIC ${PCL_LIBRARIES}
                                      ${LEMON_LIBRARIES}
                                      visualizer_cloud_and_path)
target_include_directories(Auxilary PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(Auxilary PRIVATE ${PCL_LIBRARY_DIRS})
#target_add_definitions(Auxilary PRIVATE ${PCL_DEFINITIONS})

add_library (get_navigation_point src/get_navigation_point.cpp)
target_link_libraries (get_navigation_point PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(get_navigation_point PRIVATE ${LEMON_LIBRARIES})
target_link_libraries(get_navigation_point PRIVATE Auxilary visualizer_cloud_and_path)
target_include_directories(get_navigation_point PUBLIC include ${LEMON_INCLUDE_DIR})
target_include_directories(get_navigation_point PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(get_navigation_point PRIVATE ${PCL_LIBRARY_DIRS})
#target_add_definitions(get_navigation_point PRIVATE ${PCL_DEFINITIONS})


add_executable (visualize main.cpp)
target_link_libraries (visualize PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(visualize PRIVATE ${LEMON_LIBRARIES})
target_link_libraries(visualize PRIVATE Auxilary visualizer_cloud_and_path)
target_include_directories(visualize PRIVATE include ${LEMON_INCLUDE_DIR})
target_link_libraries(visualize PRIVATE get_navigation_point)
target_include_directories(visualize PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(visualize PRIVATE ${PCL_LIBRARY_DIRS})
#target_add_definitions(visualize PRIVATE ${PCL_DEFINITIONS})


add_executable(rebuildRRTTree src/rebuildRRTTree.cpp)
target_link_libraries (rebuildRRTTree PRIVATE ${LEMON_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen
  Auxilary
  )
target_include_directories(rebuildRRTTree PRIVATE include ${LEMON_INCLUDE_DIR})
target_include_directories(rebuildRRTTree PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(rebuildRRTTree PRIVATE ${PCL_LIBRARY_DIRS})
#target_add_definitions(rebuildRRTTree PRIVATE ${PCL_DEFINITIONS})


add_library (ExportedTargetPathPlanning exported_target_path_planning.cpp)
target_link_libraries (ExportedTargetPathPlanning PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(ExportedTargetPathPlanning PRIVATE ${LEMON_LIBRARIES})
target_link_libraries(ExportedTargetPathPlanning PRIVATE Auxilary visualizer_cloud_and_path)
target_include_directories(ExportedTargetPathPlanning PRIVATE ${LEMON_INCLUDE_DIR})
target_link_libraries(ExportedTargetPathPlanning PRIVATE get_navigation_point)
target_include_directories(ExportedTargetPathPlanning PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(ExportedTargetPathPlanning PRIVATE ${PCL_LIBRARY_DIRS})
target_include_directories(ExportedTargetPathPlanning PUBLIC include )
