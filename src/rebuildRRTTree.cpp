

#include "Auxilary.h"
#include "lemon/core.h"
#include <chrono>
#include <iostream>
#include <lemon/bfs.h>
#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

bool operator==(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && rhs.z == lhs.z;
}
pcl::visualization::PCLVisualizer::Ptr
shapesVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ>
  // rgb(cloud); viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample
  // cloud");
  cloud->push_back({1, 2, 1});
  cloud->push_back({1, 2, 2});
  cloud->push_back({1, 2, 3});
  cloud->push_back({1, 2, 4});
  cloud->push_back({1, 2, 5});
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //  viewer->addCoordinateSystem(20.0);
  // viewer->initCameraParameters ();
  viewer->spinOnce(100);

  cloud->push_back({1, 2, 6});
  cloud->push_back({1, 2, 7});
  cloud->push_back({1, 2, 8});
  cloud->push_back({1, 2, 9});
  cloud->push_back({1, 2, 10});
  viewer->updatePointCloud(cloud, "sample cloud");
  viewer->spinOnce(100);
  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  //  viewer->addLine<pcl::PointXYZ> ((*cloud)[0],
  //                                     (*cloud)[cloud->size() - 1], "line");
  //  viewer->addSphere ((*cloud)[0], 0.2, 0.5, 0.5, 0.0, "sphere");
  pcl::PointXYZ point1;
  point1.x = -5;
  point1.y = 0.5;
  point1.z = 3;
  viewer->addCoordinateSystem(1.0, point1.x, point1.y, point1.z);
  viewer->addSphere(point1, 0.2, 0.5, 0.5, 0.0, "sphere1");
  pcl::PointXYZ point2;
  point2.x = -5;
  point2.y = 0.5;
  point2.z = 4;
  viewer->addCoordinateSystem(1.0, point2.x, point2.y, point2.z);
  viewer->addSphere(point2, 0.2, 0.5, 0.5, 0.0, "sphere2");
  // pcl::PointXYZ point3;
  // point3.x = -5;
  // point3.y = 0.5;
  // point3.z = 7;
  // viewer->addCoordinateSystem(1.0, point3.x, point3.y, point3.z);
  // viewer->addSphere(point3, 0.2, 1.0, 0.1, 0.1, "sphere3");
  return (viewer);
}
void update_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  float i = 6;
  while (true) {
    cloud->push_back({4, 2, i});
    std::this_thread::sleep_for(1s);
    i++;
    if (i == 10) {
      cloud->clear();
      i = 6;
    }
  }
}

/**
 * @brief This function takes the pointcloud (the map )
 * @param cloud[in] -> the Map itself
 * @param my_current_location -> the location of the drone
 *
 *
 *
 *  */

void create_new_path_given_cloud_and_current_point(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointXYZ my_current_location,
    pcl::PointCloud<pcl::PointXYZ>::Ptr visited_locations,
    lemon::ListGraph RRTGraph,
    lemon::ListGraph::NodeMap<pcl::PointXYZ> nodeToPoint)

{
  // find the closest point to current location
  pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;
  navigate_tree.setInputCloud(visited_locations);
  std::vector<int> v_closest_point(1);
  std::vector<float> v_dist_closest_point(1);
  navigate_tree.nearestKSearch(my_current_location, 1, v_closest_point,
                               v_dist_closest_point);
}

/**
 * @brief added to general Graph named "visitedGraph"
 *  the path which the drone passed
 * @param RRTGraph[in] -> the RRTGraph with all nodes
 * @param current_location[in] -> current location of the drone
 * @param  previous_location[in] -> the location of the drone before start
 * navigation
 * @param nodeToPoint[in] -> the dictionary for all node there is point
 * @param nodeToPointvisitedGraph[out] -> the dictionary for cumulative tree
 * @param visitedGraph[out] -> cumulative tree
 * */
void navigate_with_no_last_location(
    lemon::ListGraph RRTGraph, pcl::PointXYZ current_location,
    lemon::ListGraph::Node previous_location,
    lemon::ListGraph::NodeMap<pcl::PointXYZ> nodeToPoint,
    lemon::ListGraph::NodeMap<pcl::PointXYZ> &nodeToPointvisitedGraph,
    lemon::ListGraph &visitedGraph) {

  // find the closest point to current location
  pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr RRTGraphCloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (lemon::ListGraph::NodeIt it(RRTGraph); it != lemon::INVALID; ++it) {
    RRTGraphCloud->push_back(nodeToPoint[it]);
  }
  navigate_tree.setInputCloud(RRTGraphCloud);
  std::vector<int> v_closest_point(1);
  std::vector<float> v_dist_closest_point(1);
  navigate_tree.nearestKSearch(current_location, 1, v_closest_point,
                               v_dist_closest_point);

  /// get node from point
  lemon::ListGraph::Node closestPointInRRT;
  for (lemon::ListGraph::NodeIt it(RRTGraph); it != lemon::INVALID; ++it) {
    if (nodeToPoint[it] == (*RRTGraphCloud)[v_closest_point[0]]) {
      closestPointInRRT = it;
    }
  }
  /// end get node from point

  lemon::Bfs<lemon::ListGraph> bfs(RRTGraph);
  bfs.init();
  bfs.addSource(previous_location);
  bfs.start();
  if (bfs.reached(closestPointInRRT)) {
    lemon::ListGraph::Node prev = bfs.predNode(closestPointInRRT);
    while (prev != lemon::INVALID) {
      std::cout << "<-" << nodeToPoint[prev];
      visitedGraph.addEdge(prev, bfs.predNode(prev));
      // visitedGraph.addNode(prev)
      prev = bfs.predNode(prev);
    }
  }
}

void test_navigate_with_no_last_element() {
  lemon::ListGraph RRTGraph;
  lemon::ListGraph::NodeMap<pcl::PointXYZ> nodeToPoint(RRTGraph);
  lemon::ListGraph::Node startNode = RRTGraph.addNode();
  lemon::ListGraph::Node newNode2 = RRTGraph.addNode();
  lemon::ListGraph::Node newNode3 = RRTGraph.addNode();
  lemon::ListGraph::Node newNode4 = RRTGraph.addNode();
  lemon::ListGraph::Node endNode = RRTGraph.addNode();
  pcl::PointXYZ newNodePoint1 = {1, 1, 1};
  pcl::PointXYZ newNodePoint2 = {1, 1, 2};
  pcl::PointXYZ newNodePoint3 = {1, 1, 3};
  pcl::PointXYZ newNodePoint4 = {1, 1, 4};
  pcl::PointXYZ newNodePoint5 = {1, 1, 5};

  nodeToPoint[startNode] = newNodePoint1;
  nodeToPoint[newNode2] = newNodePoint2;
  nodeToPoint[newNode3] = newNodePoint3;
  nodeToPoint[newNode4] = newNodePoint4;
  nodeToPoint[endNode] = newNodePoint5;

  //         node1
  //       /       \
  //    node2      node3
  //      /          \
  //  node4           node5
  lemon::ListGraph::Edge newEdge1 = RRTGraph.addEdge(startNode, newNode2);
  lemon::ListGraph::Edge newEdge2 = RRTGraph.addEdge(startNode, newNode3);
  lemon::ListGraph::Edge newEdge3 = RRTGraph.addEdge(newNode2, newNode4);
  lemon::ListGraph::Edge newEdge4 = RRTGraph.addEdge(newNode3, endNode);
  lemon::Bfs<lemon::ListGraph> bfs(RRTGraph);
  bfs.init();
  bfs.addSource(startNode);
  bfs.start();
  if (bfs.reached(endNode)) {
    lemon::ListGraph::Node prev = bfs.predNode(endNode);
    while (prev != lemon::INVALID) {
      std::cout << "<-" << nodeToPoint[prev];
      prev = bfs.predNode(prev);
    }
  }
}

int main() {
  std::cout << "Dynamic cloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::thread update_cloud_thread(update_cloud, cloud);
  update_cloud_thread.detach();

  pcl::visualization::PCLVisualizer::Ptr viewer = shapesVis(cloud);
  while (!viewer->wasStopped()) {
    viewer->updatePointCloud(cloud, "sample cloud");
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}
