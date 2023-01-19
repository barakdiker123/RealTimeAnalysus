
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

void open_and_display_run(int index, float ScaleFactor = 0.5) {
  std::stringstream ss;
  ss << SOURCE_DIR;
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

  pcl::PointXYZ StartPoint = {-0.153138, -0.122093, 0.584504};
  pcl::PointXYZ knownPoint1 = {-0.153138, -0.122093, 0.584504};
  pcl::PointXYZ knownPoint2 = {-0.285441, -0.122198, 0.575664};
  pcl::PointXYZ knownPoint3 = {-0.0994426, -0.0767678, 0.319785};
  pcl::PointXYZ vertical;

  // crossProduct(knownPoint1 - knownPoint2, knownPoint1 - knownPoint3,
  // vertical);
  //  float alpha = -3;
  //  knownPoint1 = knownPoint1 + alpha * vertical;
  //  knownPoint2 = knownPoint2 + alpha * vertical;
  //  knownPoint3 = knownPoint3 + alpha * vertical;
  //  std::cout << vertical << "\n";

  std::list<pcl::PointXYZ> path_to_the_unknown_test;
  PathBuilder pb(ScaleFactor);

  path_to_the_unknown_test =
      pb(cloud, StartPoint, knownPoint1, knownPoint2, knownPoint3);
  for (auto point : path_to_the_unknown_test)
    std::cout << point << "->";
  visualizer_cloud_and_path(cloud, pb.ScaleFactor, path_to_the_unknown_test);
}

int main(int argc, char **argv) {

  std::cout << "If you want to change ScaleFactor you should run like this : "
               "./generalize_test [ScaleFactor between 0.1 to 1] \n  "
               "default value is 0.5"
            << std::endl;
  std::cout << "Press q to continue to next simulation " << std::endl;
  if (argc == 1)
    for (int i = 0; i < 11; i++)
      open_and_display_run(i);
  else
    for (int i = 0; i < 11; i++)
      open_and_display_run(i, std::stof(argv[1]));
}
