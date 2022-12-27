
#pragma once
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <list>
#include <thread>

#include "Auxilary.h"

void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::list<pcl::PointXYZ> path_to_unknown

);
