

#include "Auxilary.h"
#include <cmath>

/**
 * @brief This function search for additional points in the radius
 * of "PointXYZ searchPoint"
 * @param cloud -> The Cloud
 * @param searchPoint -> the point I am searching around
 * @param radius -> The radius of search
 * @return -> number of points that are in the neigbourhood of searchPoint
 */
int radiusSearch(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 pcl::PointXYZ searchPoint, float radius,
                 pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // kdtree.setInputCloud(cloud);
  //  Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  return kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance);
}
pcl::PointXYZ operator-(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  pcl::PointXYZ p3;
  p3.x = p1.x - p2.x;
  p3.y = p1.y - p2.y;
  p3.z = p1.z - p2.z;
  return p3;
}
float norm(pcl::PointXYZ p1) {
  return sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
}
pcl::PointXYZ operator/(pcl::PointXYZ p1, float d) {
  pcl::PointXYZ p3;
  if (d == 0) {
    p3.x = p1.x;
    p3.y = p1.y;
    p3.z = p1.z;
    return p3;
  }
  p3.x = p1.x / d;
  p3.y = p1.y / d;
  p3.z = p1.z / d;
  return p3;
}
/**
 * @brief copy point1 to point2
 * @param pcl::PointXYZ point1[in] -> input point
 * @param pcl::PointXYZ point2[in] -> output
 * @return -> None
 */
void copy(pcl::PointXYZ &point1, pcl::PointXYZ &point2) {
  point2.x = point1.x;
  point2.y = point1.y;
  point2.z = point1.z;
}
/**
 * @brief Given 2 vectors of size 3*1 and 3*1
 * returns the matrix multipication
 * @param pcl::PointXYZ p1[in] -> is the first point
 * @param pcl::PointXYZ p2[in] -> is the second point
 * @return float -> The matrix mul (1*3)*(3*1)
 */
float operator*(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}
pcl::PointXYZ operator*(pcl::PointXYZ p1, float a) {
  return pcl::PointXYZ(p1.x * a, p1.y * a, p1.z * a);
}

pcl::PointXYZ operator*(float a, pcl::PointXYZ p1) {
  return pcl::PointXYZ(p1.x * a, p1.y * a, p1.z * a);
}
pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2) {
  return pcl::PointXYZ(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

/**
 * @brief
 * This function build a line between @var start and @var end
 * and return all inside the line with distance at least
 * @var jump_distance
 * @param pcl::PointXYZ start -> 1 of the points to create a line
 * @param pcl::PointXYZ end -> the other point to create the line
 * @param float jump_distance -> sets the minimum distance of the point on line
 * @return -> all points on line with distance at least @var jump_distance
 */
std::vector<pcl::PointXYZ> get_points_on_line(pcl::PointXYZ start,
                                              pcl::PointXYZ end,
                                              float jump_distance) {
  std::vector<pcl::PointXYZ> points_on_line;
  pcl::PointXYZ start2end;
  start2end = end - start;
  float total_travel_line = sqrt(start2end * start2end);
  pcl::PointXYZ hat_p = start2end / total_travel_line;
  for (float i = jump_distance * 1; i < total_travel_line;
       i = i + jump_distance) {
    pcl::PointXYZ p = start + hat_p * i;
    // std::cout << p.x << "," << p.y <<"," << p.z <<std::endl;
    points_on_line.push_back(p);
  }
  return points_on_line;
}

bool is_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                       pcl::PointXYZ current_point, pcl::PointXYZ dest_point,
                       pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                       float ScaleFactor) {
  float jump_distance = 0.1 * ScaleFactor; // Magic number ?
  float radius_search = 0.25 * ScaleFactor;
  std::vector<pcl::PointXYZ> v_points_on_line =
      get_points_on_line(current_point, dest_point, jump_distance);
  for (pcl::PointXYZ &p : v_points_on_line) {
    if (radiusSearch(cloud, p, radius_search, kdtree) > 5) {
      // std::cout <<"Found Obstacle"<< p.x << "," <<p.y << "," <<p.z<<
      // std::endl;
      return false;
    }
  }
  return true;
}
// void test_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
//                          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
//   pcl::PointXYZ p1 = {-5, 0.5, 3};
//   pcl::PointXYZ p2 = {-5, 0.5, 4};
//   if (is_valid_movement(cloud, p1, p2, kdtree) == true) {
//     std::cout << " Success ! " << std::endl;
//   } else {
//     std::cout << " Failed  " << std::endl;
//   }
// }

/**
 * @brief This function finds random point on plane
 * the plane is define by those three points which are point1 point2 point3
 * @warning point1 , point2 , point3 should not be on the same line
 * preferably they should be perpedicular
 * @param point1[in] -> should be point on plane
 * @param point2[in] -> should be point on plane
 * @param point3[in] -> should be point on plane
 * @returns randomVectorOnPlane -> random point on plane !
 *
 * */
pcl::PointXYZ getRandomPointOnPlaneDefBy3Points(pcl::PointXYZ point1,
                                                pcl::PointXYZ point2,
                                                pcl::PointXYZ point3) {
  pcl::PointXYZ Vector1 = point3 - point1;
  pcl::PointXYZ Vector2 = point3 - point2;
  pcl::PointXYZ spanVector1 = point3 - point1;
  pcl::PointXYZ spanVector2 = point3 - point2;

  gram_schmint_process(/* Input -> */ Vector1, Vector2,
                       /* Output -> */ spanVector1, spanVector2);

  std::random_device
      rd; // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-50.0, 50.0);
  float RandomNum1 = dis(gen);
  float RandomNum2 = dis(gen);
  pcl::PointXYZ randomVectorOnPlane =
      RandomNum1 * spanVector1 + RandomNum2 * spanVector2 + point1;
  return randomVectorOnPlane;
}
void crossProduct(pcl::PointXYZ v_A, pcl::PointXYZ v_B, pcl::PointXYZ &c_P) {
  c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
  c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
  c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
}

void gram_schmint_process(pcl::PointXYZ v1, pcl::PointXYZ v2,
                          pcl::PointXYZ &normalized_v1,
                          pcl::PointXYZ &normalized_v2) {
  normalized_v1 = 1 / sqrtf(v1 * v1) * v1;
  normalized_v2 = v2 - (normalized_v1 * v2) * normalized_v1;
  normalized_v2 = 1 / sqrtf(normalized_v2 * normalized_v2) * normalized_v2;
}
