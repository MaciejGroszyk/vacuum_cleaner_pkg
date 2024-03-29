#include <cmath> 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;


class ScoreCoveringAlgorithm : public rclcpp::Node
{
public:
    ScoreCoveringAlgorithm() : Node("score_covering_algorithm")
    {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 30, std::bind(&ScoreCoveringAlgorithm::odomCallback, this, _1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 30, std::bind(&ScoreCoveringAlgorithm::imuCallback, this, _1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path",rclcpp::ServicesQoS());

        path_msg = nav_msgs::msg::Path();
        prev_pose_ = geometry_msgs::msg::Pose();
        prev_imu_msg = sensor_msgs::msg::Imu();

        iter_imu = 0; 
        iter_odom = 0; 

        distance = 0;
    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Reading velocity float: '%f'", msg -> angular_velocity.x);
        iter_imu++;
    }

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pathPublish(msg);
        // double x = msg->pose.pose.orientation.x;
        // double y = msg->pose.pose.orientation.y;
        // double y = msg->pose.pose.orientation.z;

        if (iter_odom > 0) 
        {
            distance += calcDistance(msg -> pose.pose.position, prev_pose_.position);
        }
        

        prev_pose_ = msg -> pose.pose;
        iter_odom++;
    }

private:
    double calcDistance(const geometry_msgs::msg::Point pose,
                        const geometry_msgs::msg::Point prev_pose) const
    {
        return sqrt(pow(pose.x - prev_pose.x, 2) + pow(pose.y - prev_pose.y, 2));
    }

    void pathPublish(nav_msgs::msg::Odometry::SharedPtr msg)
    {
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    geometry_msgs::msg::Pose prev_pose_;
    nav_msgs::msg::Path path_msg;
    sensor_msgs::msg::Imu prev_imu_msg;

    unsigned long long int iter_imu;
    unsigned long long int iter_odom;
    double distance;
};