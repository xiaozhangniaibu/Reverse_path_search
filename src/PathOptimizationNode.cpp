#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include "process_path.h"

class PathSelfIntersectionNode
{
public:
    PathSelfIntersectionNode() : nh_(), path_subscriber_()
    {
        // 订阅机器人路径主题
        path_subscriber_ = nh_.subscribe("/robot_path", 1, &PathSelfIntersectionNode::pathCallback, this);
    }

    // 从路径消息中提取Eigen类型的位姿
    std::vector<Eigen::Vector3d> extractEigenFromPath(const nav_msgs::Path &path)
    {
        std::vector<Eigen::Vector3d> poses;

        for (const auto &poseStamped : path.poses)
        {
            Eigen::Vector3d pose;
            pose[0] = poseStamped.pose.position.x;
            pose[1] = poseStamped.pose.position.y;

            // 假设方向由四元数表示
            double roll, pitch, yaw;
            tf2::Quaternion quaternion;
            tf2::fromMsg(poseStamped.pose.orientation, quaternion);
            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

            pose[2] = yaw;

            poses.push_back(pose);
        }

        return poses;
    }

    // 将Eigen类型的位姿转换为路径消息
    nav_msgs::Path convertPosesToPath(const std::vector<Eigen::Vector3d> &poses)
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now(); // 将时间戳设置为当前时间，根据需要调整
        path.header.frame_id = "map";         // 设置坐标系，根据需要调整

        for (const Eigen::Vector3d &pose : poses)
        {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position.x = pose[0];
            poseStamped.pose.position.y = pose[1] - 10; // 平移路径，方便rviz对比

            // 将角度转换为旋转矩阵
            Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

            // 将旋转矩阵转换为四元数
            Eigen::Quaterniond quaternion(rotationMatrix);
            poseStamped.pose.orientation.x = quaternion.x();
            poseStamped.pose.orientation.y = quaternion.y();
            poseStamped.pose.orientation.z = quaternion.z();
            poseStamped.pose.orientation.w = quaternion.w();

            // 设置其他位姿属性，根据需要调整

            // 将位姿添加到路径中
            path.poses.push_back(poseStamped);
        }

        return path;
    }

    // 路径回调函数
    void pathCallback(const nav_msgs::Path::ConstPtr &path_msg)
    {
        // 检查接收到的路径是否为空
        if (path_msg->poses.empty())
        {
            ROS_WARN("接收到空路径，跳过自交检查。");
            return;
        }

        // 复制路径消息
        nav_msgs::Path mutable_path_msg = *path_msg;

        // 提取Eigen类型的位姿
        std::vector<Eigen::Vector3d> poses = extractEigenFromPath(mutable_path_msg);
        std::vector<Eigen::Vector3d> poses1 = poses;

        auto start = std::chrono::high_resolution_clock::now();
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

        // 处理路径，移除自交段
        std::vector<std::pair<int, Eigen::Vector3d>> outputPath = processPath(poses, 0.1, false);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        std::chrono::duration<double, std::milli> time_cost = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time);

        std::cout << " processPath: " << time_cost.count() << " ms" << std::endl;

        // 输出处理后的路径信息
        for (const auto &vec : outputPath)
        {
            int index = vec.first;
            Eigen::Vector3d position = vec.second;
            // std::cout << "Index: " << index << ", Position: " << position.transpose() << "\n";
        }

        std::vector<Eigen::Vector3d> extractedVector;
        for (const auto &element : outputPath)
        {
            Eigen::Vector3d position = element.second;
            extractedVector.push_back(position);
        }

        std::vector<Eigen::Vector3d> extractedPoses;
        for (const auto &pair : outputPath)
        {
            int index = pair.first;
            if (index >= 0 && index < static_cast<int>(poses1.size()))
            {
                extractedPoses.push_back(poses1[index]); // 索引原路径
            }
        }

        // 发布移除自交段后的路径
        nav_msgs::Path path_without_self_intersection = convertPosesToPath(extractedPoses);
        path_pub.publish(path_without_self_intersection);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_subscriber_;
    // 发布移除自交段后的路径
    ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, true);
};

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "path_self_intersection_node");

    // 创建路径自交检查节点对象
    PathSelfIntersectionNode path_self_intersection_node;

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
