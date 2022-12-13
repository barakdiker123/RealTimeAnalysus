
#include "visualizer_cloud_and_path.hpp"

void visualizer_cloud_and_path(

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
    viewer->addSphere(point, 0.2 * ScaleFactor, 0.1, 0.2, 0.9, ss.str());
    index++;
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}
