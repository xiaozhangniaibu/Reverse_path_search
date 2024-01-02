#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

class PathPublisherNode
{
public:
    PathPublisherNode() : nh_(), listener_(), path_pub_()
    {
        // 初始化ROS节点和发布器
        ros::NodeHandle nh_private("~");
        path_pub_ = nh_.advertise<nav_msgs::Path>("/robot_path", 10, true);

        // 设置路径消息的帧 ID
        path_.header.frame_id = "map";
        path_.poses.reserve(1000); // 预留足够的空间以提高性能
    }

    void publishPath()
    {
        try
        {
            // 尝试获取base_link到map的变换
            if (listener_.canTransform("map", "base_link", ros::Time(0)))
            {
                tf::StampedTransform transform;
                listener_.lookupTransform("map", "base_link", ros::Time(0), transform);

                // 检查变换是否发生变化
                if (transform.stamp_ != last_transform_time_)
                {
                    last_transform_time_ = transform.stamp_;

                    // 创建位姿消息
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = transform.getOrigin().x();
                    pose_stamped.pose.position.y = transform.getOrigin().y();
                    pose_stamped.pose.position.z = transform.getOrigin().z();
                    pose_stamped.pose.orientation.x = transform.getRotation().x();
                    pose_stamped.pose.orientation.y = transform.getRotation().y();
                    pose_stamped.pose.orientation.z = transform.getRotation().z();
                    pose_stamped.pose.orientation.w = transform.getRotation().w();

                    // 将位姿消息添加到路径中
                    path_.poses.push_back(pose_stamped);

                    // 检查路径中是否至少有两个点
                    if (path_.poses.size() >= 2)
                    {
                        // 获取最后两个点的坐标
                        auto last_point = path_.poses[path_.poses.size() - 1].pose.position;
                        auto second_last_point = path_.poses[path_.poses.size() - 2].pose.position;
                        std::cout << "points num:" << path_.poses.size() << std::endl;
                        // ROS_INFO("Last Point: (%f, %f), Second Last Point: (%f, %f)",
                        //          last_point.x, last_point.y,
                        //          second_last_point.x, second_last_point.y);
                    }

                    // 发布路径消息
                    path_pub_.publish(path_);
                }
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    void run()
    {
        ros::Rate rate(100.0); // 调整发布频率
        while (ros::ok())
        {
            publishPath();
            rate.sleep();
            ros::spinOnce(); // 允许ROS处理回调
        }
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    ros::Time last_transform_time_;
};

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "path_publisher");

    // 创建路径发布节点对象
    PathPublisherNode path_publisher_node;

    // 运行节点
    path_publisher_node.run();

    return 0;
}
