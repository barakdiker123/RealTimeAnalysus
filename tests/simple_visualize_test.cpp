
#include <iostream>

#include "Auxilary.h"
#include "exported_target_path_planning.h"

// To use this test one should d1432.pcd sample
// For simple test
// Enter build directory and from there run the command
// ./tests/simple_visualize_test
// int main(int argc, char **argv) {
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
//  pcl::PointCloud<pcl::PointXYZ>); if
//  (pcl::io::loadPCDFile<pcl::PointXYZ>("d1432.pcd", *cloud) ==
//      -1) //* load the file
//  {
//    PCL_ERROR("Couldn't read file d1432.pcd \n");
//    return -1;
//  }
//  pcl::PointXYZ knownPoint1 = {-0.0121305, -0.0730789, 0.517468};
//  pcl::PointXYZ knownPoint2 = {-0.0177447, -0.0546628, 0.334396};
//  pcl::PointXYZ knownPoint3 = {-0.134929, -0.0606948, 0.345245};
//  pcl::PointXYZ StartPoint = {-0.134929, -0.0606948, 0.345245};
//  std::list<pcl::PointXYZ> path_to_the_unknown_test;
//  PathBuilder pb(1);
//
//  path_to_the_unknown_test =
//      pb(cloud, StartPoint, knownPoint1, knownPoint2, knownPoint3);
//  for (auto point : path_to_the_unknown_test)
//    std::cout << point << "->";
//  visualizer_cloud_and_path(cloud, 0.2, path_to_the_unknown_test);
//}

int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("real_time_pcd_27122022.pcd",
                                            *cloud) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file d1432.pcd \n");
        return -1;
    }
    float ScaleFactor = 0.2;
    pcl::PointXYZ StartPoint = {-0.153138, -0.122093, 0.584504};
    pcl::PointXYZ knownPoint1 = {-0.153138, -0.122093, 0.584504};
    pcl::PointXYZ knownPoint2 = {-0.285441, -0.122198, 0.575664};
    pcl::PointXYZ knownPoint3 = {-0.0994426, -0.0767678, 0.319785};
    pcl::PointXYZ vertical;

    crossProduct(knownPoint1 - knownPoint2, knownPoint1 - knownPoint3,
                 vertical);
    float alpha = -3;
    knownPoint1 = knownPoint1 + alpha * vertical;
    knownPoint2 = knownPoint2 + alpha * vertical;
    knownPoint3 = knownPoint3 + alpha * vertical;
    std::cout << vertical << "\n";

    std::list<pcl::PointXYZ> path_to_the_unknown_test;
    PathBuilder pb(ScaleFactor);

    path_to_the_unknown_test =
        pb(cloud, StartPoint, knownPoint1, knownPoint2, knownPoint3);
    for (auto point : path_to_the_unknown_test) std::cout << point << "->";
    visualizer_cloud_and_path(cloud, ScaleFactor, path_to_the_unknown_test);
}
