#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using std::placeholders::_1;

class ScoreCoveringAlgorithm : public rclcpp::Node
{
public:
    ScoreCoveringAlgorithm() : Node("score_covering_algorithm")
    {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 30, std::bind(&ScoreCoveringAlgorithm::odomCallback, this, _1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path",rclcpp::ServicesQoS());

        path_msg = nav_msgs::msg::Path();
    }


    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
    {

        // double x = msg->pose.pose.orientation.x;
        // double y = msg->pose.pose.orientation.y;
        // double y = msg->pose.pose.orientation.z;

        
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose = msg->pose.pose;

        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        path_msg.poses.push_back(pose_msg);
        path_publisher_ -> publish(path_msg);
    }

    void main()
    {
        
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::Path path_msg;
};