

#include "get_navigation_point.hpp"
#include "Auxilary.h"
#include "lemon/core.h"
#include "lemon/list_graph.h"

bool operator==(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && rhs.z == lhs.z;
}

void get_navigation_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           pcl::PointXYZ navigate_starting_point,
                           pcl::PointXYZ knownPoint1, pcl::PointXYZ knownPoint2,
                           pcl::PointXYZ knownPoint3, float ScaleFactor,
                           std::list<pcl::PointXYZ> &path_to_the_unknown,
                           std::vector<Edge> &v_edges,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr RRTCloud) {
  lemon::ListGraph RRTGraph;
  lemon::ListGraph::NodeMap<pcl::PointXYZ> nodeToPoint(RRTGraph);
  // std::unordered_map<pcl::PointXYZ, lemon::ListGraph::Node> pointToNode;
  // points2.pcd
  // pcl::PointXYZ knownPoint1 = {-0.264355 + 0.4, -0.105879 + 0.1, 0.166656};
  // pcl::PointXYZ knownPoint2 = {-0.281147, -0.032053 + 0.1, -0.252866};
  // pcl::PointXYZ knownPoint3 = {-0.661326, -0.038876 + 0.1, -0.242295};
  // points3.pcd
  // pcl::PointXYZ knownPoint1 = {-0.152039, -0.112927 , 0.521806};
  // pcl::PointXYZ knownPoint2 = {-0.0388202, 0.0144199 , -0.150134};
  // pcl::PointXYZ knownPoint3 = {0.0739677 , 0.0088157 , -0.170298};

  //  2.98735 -0.312701 0.234717
  // 2.15991 -0.303712 0.337741
  // 3.11331 -0.574892 0.515388

  // pcl::PointXYZ knownPoint1 = {2.98735, -0.312701, 0.234717};
  // pcl::PointXYZ knownPoint3 = {2.15991, -0.303712, 0.337741};
  // pcl::PointXYZ knownPoint2 = {3.11331, -0.574892, 0.515388};

  // pcl::visualization::PCLVisualizer::Ptr viewer = shapesVis(cloud);
  //  test RRT
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // pcl::PointXYZ navigate_starting_point = {0.114433, -0.0792644 + 0.0,
  //                                          0.238077};
  // navigate_starting_point.x = knownPoint1.x;
  // navigate_starting_point.y = knownPoint1.y;
  // navigate_starting_point.z = knownPoint1.z;
  //  std::vector<Edge> v_edges;
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr RRTCloud(
  //      new pcl::PointCloud<pcl::PointXYZ>);
  RRTCloud->push_back(navigate_starting_point);
  lemon::ListGraph::Node firstNode = RRTGraph.addNode();
  nodeToPoint[firstNode] = navigate_starting_point;

  //  std::random_device
  //      rd; // Will be used to obtain a seed for the random number engine
  //  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with
  //  rd() std::uniform_real_distribution<> dis(-30.0, 30.0); float jump = 0.5;

  float jump = 0.8 * ScaleFactor;
  for (int i = 0; i < 2000; i++) {
    pcl::PointXYZ point_rand = getRandomPointOnPlaneDefBy3Points(
        knownPoint1, knownPoint2, knownPoint3);
    pcl::KdTreeFLANN<pcl::PointXYZ> navigate_tree1;
    navigate_tree1.setInputCloud(RRTCloud);
    std::vector<int> v_closest_point1(1);
    std::vector<float> v_dist_closest_point1(1);
    if (navigate_tree1.nearestKSearch(point_rand, 1, v_closest_point1,
                                      v_dist_closest_point1)) {
      pcl::PointXYZ point_new =
          (*RRTCloud)[v_closest_point1[0]] +
          jump * (point_rand - (*RRTCloud)[v_closest_point1[0]]) /
              (sqrt((point_rand - (*RRTCloud)[v_closest_point1[0]]) *
                    (point_rand - (*RRTCloud)[v_closest_point1[0]])));
      if (is_valid_movement(cloud, (*RRTCloud)[v_closest_point1[0]], point_new,
                            kdtree, ScaleFactor)) {
        RRTCloud->push_back(point_new);
        Edge e = {point_new, (*RRTCloud)[v_closest_point1[0]]};
        v_edges.push_back(e);

        lemon::ListGraph::Node newNode = RRTGraph.addNode();
        // pointToNode[point_new] = newNode;
        nodeToPoint[newNode] = point_new;

        lemon::ListGraph::Node closestPointInRRT;
        for (lemon::ListGraph::NodeIt it(RRTGraph); it != lemon::INVALID;
             ++it) {
          if (nodeToPoint[it] == (*RRTCloud)[v_closest_point1[0]]) {
            closestPointInRRT = it;
          }
        }
        lemon::ListGraph::Edge newEdge =
            RRTGraph.addEdge(newNode, closestPointInRRT);
      }
    } else {
      std::cout << "cannot find nearest Node!!" << std::endl;
    }
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  int index = 1;
  for (Edge edge : v_edges) {
    std::stringstream ss;
    ss << "navigate_line" << index;
    // viewer->addLine<pcl::PointXYZ>((edge.p_point1), (edge.p_point2),
    // ss.str());
    index++;
  }
  index = 1;
  for (pcl::PointXYZ p : *RRTCloud) {
    std::stringstream ss;
    ss << "PointNavigate" << index;
    // viewer->addSphere(p, 0.2 * ScaleFactor, 0.9, 0.2, 0.0, ss.str());
    index++;
  }

  std::cout << v_edges.size() << std::endl;
  std::cout << (*RRTCloud).size() << std::endl;
  for (pcl::PointXYZ p : *RRTCloud) {
    // std::cout << p.x << "," << p.y << "," << p.z;
  }

  // Checks whether the number of points in
  // RRT Graph is Bigger than 3
  // if so exit
  if (lemon::countNodes(RRTGraph) < 50) {
    return;
  }

  // Random Vertices from the RRTGraph!
  std::random_device os_seed;
  const uint_least32_t seed = os_seed();

  std::mt19937 generator(seed);
  std::uniform_int_distribution<uint_least32_t> distribute(
      1, lemon::countNodes(RRTGraph) - 2);
  lemon::ListGraph::NodeIt n(RRTGraph);
  for (int i = 0; i < distribute(generator); i++) {
    ++n;
  }
  lemon::ListGraph::Node get_random_node_from_RRT = n;
  // End Random Vertices from the RRTGraph!

  // BFS
  lemon::Bfs<lemon::ListGraph> bfs(RRTGraph);
  bfs.init();
  bfs.addSource(firstNode);
  bfs.start();

  if (bfs.reached(get_random_node_from_RRT)) {
    path_to_the_unknown.emplace_front(nodeToPoint[get_random_node_from_RRT]);
    // std::cout << nodeToPoint[get_random_node_from_RRT];
    lemon::ListGraph::Node prev = bfs.predNode(get_random_node_from_RRT);
    while (prev != lemon::INVALID) {
      // std::cout << "<-" << nodeToPoint[prev];
      path_to_the_unknown.emplace_front(nodeToPoint[prev]);
      prev = bfs.predNode(prev);

      std::stringstream ss;
      ss << "PointNavigatePath" << index;
      // viewer->addSphere(nodeToPoint[prev], 0.2 * ScaleFactor, 0.1, 0.2, 0.9,
      //                   ss.str());
      index++;
    }
  }

  // viewer->addCoordinateSystem(1.0, navigate_starting_point.x,
  //                             navigate_starting_point.y,
  //                             navigate_starting_point.z);

  // pcl::PointXYZ p_helper{0.0867836,0.031304,0.0313494};
  //  pcl::PointXYZ p_helper{0.152039, -0.112927,0.521806};
  // pcl::PointXYZ p_helper{-0.804435 , -0.00937385 , 0.289798};
  // Print Path
  // viewer->addSphere(pcl::PointXYZ(-0.804435, -0.00937385, 0.289798),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 1.0, "SHPERE121");
  // viewer->addSphere(pcl::PointXYZ(-0.684641, -0.0372174, 0.37567),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.9, "SHPERE122");
  // viewer->addSphere(pcl::PointXYZ(-0.568991, -0.0657816, 0.466824),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.8, "SHPERE123");
  // viewer->addSphere(pcl::PointXYZ(-0.460346, -0.0953637, 0.565926),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.7, "SHPERE124");
  // viewer->addSphere(pcl::PointXYZ(-0.342639, -0.123583, 0.65452),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.6, "SHPERE125");
  // viewer->addSphere(pcl::PointXYZ(-0.193642, -0.138288, 0.663672),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.5, "SHPERE126");
  // viewer->addSphere(pcl::PointXYZ(-0.152039, -0.112927, 0.521806),
  //                   0.15 * ScaleFactor, 1.0, 1.0, 0.4, "SHPERE127");
  // viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.15 * ScaleFactor, 0.3, 0.5,
  // 0.5,
  //                   "SHPERE127asdfasDolevf");

  //  pcl::PointXYZ knownPoint4 = {-0.264355 + 0.6, -0.105879 + 0.1, 0.166656};
  //  viewer->addSphere(pcl::PointXYZ(0.320001, 0.028691, -0.409289),
  //                    0.3 * ScaleFactor, 0.15, 0.5, 0.5,
  //                    "SHPERE127asdfasDolevf");
  //  viewer->addSphere(knownPoint4, 0.15 * ScaleFactor, 0.3, 0.5, 0.5,
  //                    "SHPERE127asdfasDolevf1");
  // viewer->addSphere(knownPoint1, 0.5 * ScaleFactor, 1, 0, 0,
  //                   "SHPERE127asdfasDolevf1");

  // viewer->addSphere(knownPoint2, 0.5 * ScaleFactor, 0, 1, 0,
  //                   "SHPERE127qsdfasDolevf1");

  // viewer->addSphere(knownPoint3, 0.5 * ScaleFactor, 0, 0, 1,
  //                   "SHPERE127aadfasDolevf1");
  //--------------------
  // -----Main loop-----
  //--------------------
  // If the Path is less than some magic number ie 10 than rerun
  if (path_to_the_unknown.size() < 10) {
    return;
  }
  //  std::thread viewer_thread(
  //      [=](pcl::visualization::PCLVisualizer::Ptr viewer) {
  //        while (!viewer->wasStopped()) {
  //          viewer->spinOnce(100);
  //          std::this_thread::sleep_for(100ms);
  //        }
  //      },
  //      viewer);
  //  viewer_thread.detach();

  // while (!viewer->wasStopped()) {
  //   viewer->spinOnce(100);
  //   std::this_thread::sleep_for(100ms);
  // }
}
