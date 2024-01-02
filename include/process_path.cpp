#include "process_path.h"

using namespace boost;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// 定义二维点类型
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
// 定义包围盒类型
typedef bg::model::box<point> box;
// 定义值类型，存储点和索引
typedef std::pair<point, int> value;

// 定义图类型
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Eigen::Vector2d, boost::property<boost::edge_weight_t, double>> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

/**
 * @brief 计算两点之间的欧几里得距离
 * @param p1 第一个点的坐标
 * @param p2 第二个点的坐标
 * @return 两点之间的欧几里得距离
 */
double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    return (p1 - p2).norm();
}

// 定义A*算法所需的启发式函数类
class distance_heuristic : public boost::astar_heuristic<Graph, double>
{
public:
    distance_heuristic(const Graph &g, Vertex goal)
        : m_g(g), m_goal(goal) {}

    double operator()(Vertex v)
    {
        auto vertex_x = m_g[v].x();
        auto vertex_y = m_g[v].y();

        return distance(Eigen::Vector2d(vertex_x, vertex_y),
                        Eigen::Vector2d(m_g[m_goal].x(), m_g[m_goal].y()));
    }

private:
    const Graph &m_g;
    Vertex m_goal;
};

/**
 * @brief 处理路径并返回处理后的路径
 * @param path 输入路径，以Eigen::Vector3d表示，包含三维坐标信息
 * @param thresholdDist 用户指定的距离阈值，小于该阈值的点之间将会建立边
 * @param useAStar 是否使用A*算法，如果为false则使用Dijkstra算法
 * @return 返回处理后的路径，每个路径点包括序号和带有方向信息的三维坐标
 */
std::vector<std::pair<int, Eigen::Vector3d>> processPath(const std::vector<Eigen::Vector3d> &path, double thresholdDist, bool useAStar)
{
    std::vector<std::pair<int, Eigen::Vector3d>> processedPath;

    // 反转路径，以创建返回路径
    std::vector<Eigen::Vector3d> reversedPath = path;
    std::reverse(reversedPath.begin(), reversedPath.end());

    // 为高效的空间查询构建R-tree
    std::vector<value> rtree_values;
    for (int i = 0; i < reversedPath.size(); ++i)
    {
        rtree_values.push_back(std::make_pair(point(reversedPath[i].x(), reversedPath[i].y()), i));
    }

    bgi::rtree<value, bgi::quadratic<16>> rtree(rtree_values.begin(), rtree_values.end());

    // 构建图
    Graph g(reversedPath.size());
    property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, g);

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    for (int i = 0; i < reversedPath.size(); ++i)
    {
        if (reversedPath[i].x() == reversedPath[i + 1].x() && reversedPath[i].y() == reversedPath[i + 1].y() && i > 0)
        {
            continue;
        }

        box query_box(point(reversedPath[i].x() - thresholdDist, reversedPath[i].y() - thresholdDist),
                      point(reversedPath[i].x() + thresholdDist, reversedPath[i].y() + thresholdDist));

        std::vector<value> result;
        rtree.query(bgi::intersects(query_box), std::back_inserter(result));

        for (const auto &pair : result)
        {
            int j = pair.second;
            double dist = distance(Eigen::Vector2d(reversedPath[i].x(), reversedPath[i].y()),
                                   Eigen::Vector2d(reversedPath[j].x(), reversedPath[j].y()));
            if ((dist < thresholdDist) && j > i + 1)
            {
                add_edge(i, j, dist, g);
            }
        }
        double dist = distance(Eigen::Vector2d(reversedPath[i].x(), reversedPath[i].y()),
                               Eigen::Vector2d(reversedPath[i + 1].x(), reversedPath[i + 1].y()));
        add_edge(i, i + 1, dist, g);
    }

    std::chrono::duration<double, std::milli> time_cost = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time);

    // std::cout << " 添加边耗时: " << time_cost.count() << " 毫秒" << std::endl;

    // 计算最短路径
    std::vector<Vertex> predecessors(num_vertices(g));
    std::vector<double> distances(num_vertices(g));

    if (useAStar)
    {
        // 使用A*算法
        auto visitor = boost::make_astar_visitor(boost::record_predecessors(predecessors, boost::on_tree_edge()));
        boost::astar_search(g, 0, distance_heuristic(g, reversedPath.size() - 1),
                            boost::predecessor_map(predecessors.data())
                                .distance_map(distances.data())
                                .visitor(visitor));
    }
    else
    {
        // 使用Dijkstra算法
        boost::dijkstra_shortest_paths(g, 0,
                                       boost::predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g)))
                                           .distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g))));
    }

    // 构建最短路径
    int current = reversedPath.size() - 1;
    while (current != 0)
    {
        Eigen::Vector3d currentPoint(reversedPath[current].x(), reversedPath[current].y(), 0);
        Eigen::Vector3d previousPoint(reversedPath[predecessors[current]].x(), reversedPath[predecessors[current]].y(), 0);

        Eigen::Vector3d direction = currentPoint - previousPoint;
        double yaw = atan2(direction.y(), direction.x());

        if (current == reversedPath.size() - 1)
        {
            processedPath.push_back(std::make_pair(reversedPath.size() - 1 - current,
                                                   Eigen::Vector3d(reversedPath[current].x(), reversedPath[current].y(), yaw)));
        }

        processedPath.push_back(std::make_pair(reversedPath.size() - 1 - predecessors[current],
                                               Eigen::Vector3d(reversedPath[predecessors[current]].x(),
                                                               reversedPath[predecessors[current]].y(), yaw)));

        current = predecessors[current];
    }

    return processedPath;
}
