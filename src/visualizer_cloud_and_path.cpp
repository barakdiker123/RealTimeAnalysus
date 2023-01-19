
#include "visualizer_cloud_and_path.hpp"

Visualized Visualized::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  return *this;
}
Visualized Visualized::set_path(std::list<pcl::PointXYZ> path_to_unknown) {
  int index = 0;
  for (auto &point : path_to_unknown) {
    std::stringstream ss;
    ss << "PointNavigatePath" << index;
    viewer->addSphere(point, 0.25 * ScaleFactor, 0.1, 0.2, 0.9, ss.str());
    index++;
  }
  if (!path_to_unknown.empty()) {
    viewer->addSphere(path_to_unknown.front(), 0.25 * ScaleFactor + 0.01, 0.9,
                      0.2, 0.2, "Starting Point");
  }
  return *this;
}
Visualized Visualized::set_scale_factor(float ScaleFactor) {
  this->ScaleFactor = ScaleFactor;
  return *this;
}
void Visualized::run() {
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}

Visualized Visualized::set_edges(std::vector<Edge> &v_edges) {}
Visualized Visualized::set_RRTCloud(pcl::PointCloud<pcl::PointXYZ> &RRTCloud) {}

void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float ScaleFactor,
    std::list<pcl::PointXYZ> path_to_unknown

) {

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ>
  // rgb(cloud); viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample
  // cloud");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  int index = 0;
  for (auto &point : path_to_unknown) {
    std::stringstream ss;
    ss << "PointNavigatePath" << index;
    viewer->addSphere(point, 0.25 * ScaleFactor, 0.1, 0.2, 0.9, ss.str());
    index++;
  }
  if (!path_to_unknown.empty()) {
    viewer->addSphere(path_to_unknown.front(), 0.25 * ScaleFactor + 0.01, 0.9,
                      0.2, 0.2, "Starting Point");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}
