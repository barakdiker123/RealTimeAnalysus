
#include <iostream>

#include "Auxilary.h"
#include "config.h"
#include "exported_target_path_planning.h"

// For Automation
//  std::stringstream ss;
//  ss << BUILD_DIR;
//  ss << "/MyData";
//  ss << ".pcd";
int main(int argc, char **argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],
                                          *cloud) == -1) //* load the file
  {
    PCL_ERROR("For Using the program run:\n ./simple_visualize_test [PATH TO "
              "THE POINT CLOUD]");
    return -1;
  }
  // ScaleFactor is the initial Ball Size (You should start Big and it'll shrink
  // to the fitted size )
  float ScaleFactor = 0.5;
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
  visualizer_cloud_and_path(cloud, ScaleFactor, path_to_the_unknown_test);
}
