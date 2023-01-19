
#include "Auxilary.h"
#include "config.h"
#include "exported_target_path_planning.h"
#include "visualizer_cloud_and_path.hpp"
#include <iostream>

pcl::PointXYZ getCenterOfMass(pcl::PointCloud<pcl::PointXYZ> cloud) {
  pcl::PointXYZ center_of_mass_point = {0, 0, 0};
  for (auto point : cloud) {
    center_of_mass_point = center_of_mass_point + point;
  }
  center_of_mass_point = 1 / (float)cloud.size() * center_of_mass_point;
  return center_of_mass_point;
}

void open_and_display_run(int index) {
  std::stringstream ss;
  ss << BUILD_DIR;
  ss << "/pcd_test_data"
     << "/"
     << "pointData";
  ss << index << ".pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(),
                                          *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return;
  } else {
    std::cout << "File has been found " << index << std::endl;
  }
}

int main() {
  for (int i = 0; i < 11; i++)
    open_and_display_run(i);
}

// #cmakedefine BUILD_DIR @BUILD_DIR@
// set(BUILD_DIR \"${CMAKE_CURRENT_SOURCE_DIR}\")
// configure_file(PathConfig.h.in PathConfig.h)
