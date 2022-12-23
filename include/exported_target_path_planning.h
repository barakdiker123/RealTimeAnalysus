
#pragma once

#include "Auxilary.h"
#include "get_navigation_point.hpp"
//#include "lemon/core.h"
#include "visualizer_cloud_and_path.hpp"
//#include <chrono>
//#include <fstream>
//#include <iostream>
//#include <lemon/bfs.h>
//#include <lemon/list_graph.h>
//#include <lemon/maps.h>
//#include <list>
//#include <pcl/common/common_headers.h>
//#include <pcl/console/parse.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <cstddef>
#include <random>
#include <sstream>
#include <thread>

class PathBuilder {
private:
  float ScaleFactor = 0.2;
  float byHowMuchChangeScaleFactor = 0.1;
  std::size_t howManyTimesTriesEachScaleFactor = 10;
  std::size_t howLongIsAValidPath = 10;
  bool debug = true;

public:
  PathBuilder(float ScaleFactor) : ScaleFactor(ScaleFactor) {}
  PathBuilder() {}
  /**
   * @brief This function takes parameters and
   * return the path to the unknown via path_to_the_unknown
   * variable
   * @param cloud[in] -> Entire model
   * @param StartPoint[in] -> Start Point
   * @param knownPoint1[in] -> Points needed for creating the plane for RRT
   * @returns path_to_the_unknown[out] -> The returned path to the unknown
   * */
  std::list<pcl::PointXYZ> operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointXYZ StartPoint,
                                      pcl::PointXYZ knownPoint1,
                                      pcl::PointXYZ knownPoint2,
                                      pcl::PointXYZ knownPoint3

  );
  /**
   * @brief This function takes parameters and
   * return the path to the unknown via path_to_the_unknown
   * variable
   * @param cloud[in] -> Entire model
   * @param StartPoint[in] -> Start Point
   * @param knownPoint1[in] -> Points needed for creating the plane for RRT
   * @param path_to_the_unknown[out] -> The returned path to the unknown
   * */
  void operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointXYZ StartPoint, pcl::PointXYZ knownPoint1,
                  pcl::PointXYZ knownPoint2, pcl::PointXYZ knownPoint3,
                  std::list<pcl::PointXYZ> &path_to_the_unknown);
  /**
   * @brief This function takes parameters and
   * return the path to the unknown via path_to_the_unknown
   * variable
   * @override create text file
   * @param cloud[in] -> Entire model
   * @param StartPoint[in] -> Start Point
   * @param knownPoint1[in] -> Points needed for creating the plane for RRT
   * @param path_to_the_unknown[out] -> The returned path to the unknown
   * */
  void operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointXYZ StartPoint, pcl::PointXYZ knownPoint1,
                  pcl::PointXYZ knownPoint2, pcl::PointXYZ knownPoint3,
                  std::string location_file_path_to_the_unknown);
};
