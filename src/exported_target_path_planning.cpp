
//#include "Auxilary.h"
//#include "get_navigation_point.hpp"
////#include "lemon/core.h"
//#include "visualizer_cloud_and_path.hpp"
////#include <chrono>
////#include <fstream>
////#include <iostream>
////#include <lemon/bfs.h>
////#include <lemon/list_graph.h>
////#include <lemon/maps.h>
////#include <list>
////#include <pcl/common/common_headers.h>
////#include <pcl/console/parse.h>
////#include <pcl/features/normal_3d.h>
////#include <pcl/io/pcd_io.h>
////#include <pcl/visualization/pcl_visualizer.h>
//#include <random>
//#include <sstream>
//#include <thread>

#include "exported_target_path_planning.h"
/**
 * @brief This function takes parameters and
 * return the path to the unknown via path_to_the_unknown
 * variable
 * @param cloud[in] -> Entire model
 * @param StartPoint[in] -> Start Point
 * @param knownPoint1[in] -> Points needed for creating the plane for RRT
 * @returns path_to_the_unknown[out] -> The returned path to the unknown
 * */
std::list<pcl::PointXYZ>
PathBuilder::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointXYZ StartPoint, pcl::PointXYZ knownPoint1,
                        pcl::PointXYZ knownPoint2, pcl::PointXYZ knownPoint3

) {
  std::list<pcl::PointXYZ> path_to_the_unknown;

  int how_many_times_until_change_scalefactor =
      howManyTimesTriesEachScaleFactor;
  while (path_to_the_unknown.size() < howLongIsAValidPath) {
    path_to_the_unknown.clear();

    std::vector<Edge> v_edges;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RRTCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    get_navigation_points(cloud, StartPoint, knownPoint1, knownPoint2,
                          knownPoint3, ScaleFactor, path_to_the_unknown,
                          v_edges, RRTCloud);

    if (how_many_times_until_change_scalefactor == 0) {
      ScaleFactor = ScaleFactor * (1 - byHowMuchChangeScaleFactor);
      how_many_times_until_change_scalefactor =
          howManyTimesTriesEachScaleFactor;
    }
    how_many_times_until_change_scalefactor--;
  }
  if (debug)
    for (auto point : path_to_the_unknown) {
      std::cout << point.x << " " << point.y << " " << point.z << "->";
    }
  return path_to_the_unknown;
}
/**
 * @brief This function takes parameters and
 * return the path to the unknown via path_to_the_unknown
 * variable
 * @param cloud[in] -> Entire model
 * @param StartPoint[in] -> Start Point
 * @param knownPoint1[in] -> Points needed for creating the plane for RRT
 * @param path_to_the_unknown[out] -> The returned path to the unknown
 * */
void PathBuilder::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointXYZ StartPoint,
                             pcl::PointXYZ knownPoint1,
                             pcl::PointXYZ knownPoint2,
                             pcl::PointXYZ knownPoint3,
                             std::list<pcl::PointXYZ> &path_to_the_unknown) {
  int how_many_times_until_change_scalefactor =
      howManyTimesTriesEachScaleFactor;
  while (path_to_the_unknown.size() < howLongIsAValidPath) {
    path_to_the_unknown.clear();

    std::vector<Edge> v_edges;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RRTCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    get_navigation_points(cloud, StartPoint, knownPoint1, knownPoint2,
                          knownPoint3, ScaleFactor, path_to_the_unknown,
                          v_edges, RRTCloud);

    if (how_many_times_until_change_scalefactor == 0) {
      ScaleFactor = ScaleFactor * (1 - byHowMuchChangeScaleFactor);
      how_many_times_until_change_scalefactor =
          howManyTimesTriesEachScaleFactor;
    }
    how_many_times_until_change_scalefactor--;
  }
  if (debug)
    for (auto point : path_to_the_unknown) {
      std::cout << point.x << " " << point.y << " " << point.z << "->";
    }
}
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
void PathBuilder::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointXYZ StartPoint,
                             pcl::PointXYZ knownPoint1,
                             pcl::PointXYZ knownPoint2,
                             pcl::PointXYZ knownPoint3,
                             std::string location_file_path_to_the_unknown) {
  std::list<pcl::PointXYZ> path_to_the_unknown;
  int how_many_times_until_change_scalefactor =
      howManyTimesTriesEachScaleFactor;
  while (path_to_the_unknown.size() < howLongIsAValidPath) {
    path_to_the_unknown.clear();

    std::vector<Edge> v_edges;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RRTCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    get_navigation_points(cloud, StartPoint, knownPoint1, knownPoint2,
                          knownPoint3, ScaleFactor, path_to_the_unknown,
                          v_edges, RRTCloud);

    if (how_many_times_until_change_scalefactor == 0) {
      ScaleFactor = ScaleFactor * (1 - byHowMuchChangeScaleFactor);
      how_many_times_until_change_scalefactor =
          howManyTimesTriesEachScaleFactor;
    }
    how_many_times_until_change_scalefactor--;
  }
  if (debug)
    for (auto point : path_to_the_unknown)
      cout << point.x << point.y << point.z << "->";
  std::ofstream file_of_path_to_the_unknown;
  // file_of_path_to_the_unknown.open("drone_destinations1.txt");
  file_of_path_to_the_unknown.open(location_file_path_to_the_unknown.c_str());
  for (auto point : path_to_the_unknown) {
    file_of_path_to_the_unknown << point.x << " " << point.y << " " << point.z
                                << "\n";
  }
  file_of_path_to_the_unknown.close();
}
