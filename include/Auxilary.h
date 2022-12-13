
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>

const float ScaleFactor = 0.5;

using namespace std::chrono_literals;

struct Edge {
  pcl::PointXYZ p_point1;
  pcl::PointXYZ p_point2;
};

int radiusSearch(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 pcl::PointXYZ searchPoint, float radius,
                 pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);

pcl::PointXYZ operator-(pcl::PointXYZ p1, pcl::PointXYZ p2);
float norm(pcl::PointXYZ p1);
pcl::PointXYZ operator/(pcl::PointXYZ p1, float d);

void copy(pcl::PointXYZ &point1, pcl::PointXYZ &point2);

/**
 * @brief Given 2 vectors of size 3*1 and 3*1
 * returns the matrix multipication
 * @param pcl::PointXYZ p1[in] -> is the first point
 * @param pcl::PointXYZ p2[in] -> is the second point
 * @return float -> The matrix mul (1*3)*(3*1)
 */
float operator*(pcl::PointXYZ p1, pcl::PointXYZ p2);
pcl::PointXYZ operator*(pcl::PointXYZ p1, float a);
pcl::PointXYZ operator*(float a, pcl::PointXYZ p1);
pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2);

// Hashable function for pcl::PointXYZ
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
std::vector<pcl::PointXYZ>
get_points_on_line(pcl::PointXYZ start, pcl::PointXYZ end, float jump_distance);

bool is_valid_movement(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                       pcl::PointXYZ current_point, pcl::PointXYZ dest_point,
                       pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);

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
                                                pcl::PointXYZ point3);
