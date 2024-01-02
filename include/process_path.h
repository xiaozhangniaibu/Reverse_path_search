// process_path.h
#ifndef PROCESS_PATH_H
#define PROCESS_PATH_H

#include <vector>
#include <Eigen/Dense>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <chrono>
std::vector<std::pair<int, Eigen::Vector3d>> processPath(const std::vector<Eigen::Vector3d> &path, double thresholdDist, bool useAStar);

#endif // PROCESS_PATH_H
