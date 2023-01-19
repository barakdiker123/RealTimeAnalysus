
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

/**
 * @brief This function visualise the point cloud
 * and visualize a path (usually received from @path_planning)
 * @param cloud[in] -> The model of the environment
 * @param ScaleFactor[in] -> The Scale Factor
 * @oaram path_to_unknown[in] -> list of point you want to see
 * */
void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::list<pcl::PointXYZ> path_to_unknown

);

/**
 * @brief same as visualizer_cloud_and_path
 * implementation in OOP
 *
 * Usage :
 * Visualizer visualizer();
 * visualizer.set_cloud(Enter Some Cloud).set_path(enter some
 * path).set_scale_factor(size) visualizer.run()
 *
 *
 *  */
struct Visualized {
  /**
   * @var viewer is the interface of the bigger library
   * for more adaptation read PCL
   * @var ScaleFactor -> We rarely know the scale that's why I keep It in inner
   * state
   * */
  pcl::visualization::PCLVisualizer::Ptr viewer;
  float ScaleFactor = 0.5;

  Visualized() : viewer(new pcl::visualization::PCLVisualizer("3D Viewer")) {
    viewer->setBackgroundColor(0, 0, 0);
  }
  Visualized set_scale_factor(float ScaleFactor);
  Visualized set_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  Visualized set_path(std::list<pcl::PointXYZ> path_to_unknown);
  Visualized set_edges(std::vector<Edge> &v_edges);
  Visualized set_RRTCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr RRTCloud);
  void run();
};
