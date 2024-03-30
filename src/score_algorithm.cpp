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
        // prev_odom_msg_ = nav_msgs::msg::Odometry();
        // prev_imu_msg_ = sensor_msgs::msg::Imu();

        iter_imu = 0; 
        iter_odom = 0; 

        path_length = 0;
        cumulative_rotation_change = 0;
    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Reading velocity float: '%f'", msg -> angular_velocity.x);
        prev_imu_msg_ = msg;
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
            path_length += calcPathLength(msg -> pose.pose.position, prev_odom_msg_ -> pose.pose.position);
            cumulative_rotation_change += calcRotationChange(msg -> twist.twist, prev_odom_msg_ -> twist.twist);
        }
        RCLCPP_INFO(this->get_logger(), "Actual path_length: '%f'",path_length);
        

        prev_odom_msg_ = msg;
        iter_odom++;
    }

private:
    double calcPathLength(const geometry_msgs::msg::Point pose,
                        const geometry_msgs::msg::Point prev_pose) const
    {
        return sqrt(pow(pose.x - prev_pose.x, 2) + pow(pose.y - prev_pose.y, 2));
    }

    double calcRotationChange(  const geometry_msgs::msg::Twist msg,
                                const geometry_msgs::msg::Twist prev_msg) const
    {
        return abs(prev_msg.angular.z - msg.angular.z);
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
    
    nav_msgs::msg::Path path_msg;

    // prev msgs
    nav_msgs::msg::Odometry::SharedPtr prev_odom_msg_;
    sensor_msgs::msg::Imu::SharedPtr prev_imu_msg_;

    //sub iterators
    unsigned long long int iter_imu;
    unsigned long long int iter_odom;

    //data
    double path_length;
    double cumulative_rotation_change;
};