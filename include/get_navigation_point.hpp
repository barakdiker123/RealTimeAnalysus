
#pragma once
#include <lemon/bfs.h>
#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <list>
#include <random>
#include <sstream>
#include <thread>

#include "lemon/core.h"
#include "visualizer_cloud_and_path.hpp"

void get_navigation_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           pcl::PointXYZ navigate_starting_point,
                           pcl::PointXYZ knownPoint1, pcl::PointXYZ knownPoint2,
                           pcl::PointXYZ knownPoint3,
                           std::list<pcl::PointXYZ> &path_to_the_unknown,
                           float ScaleFactor);
