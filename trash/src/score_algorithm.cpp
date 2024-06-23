#include <cmath> 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp> 
#include "draw_robot_pose_on_map.cpp"


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

        /* The `path_length` variable in the `ScoreCoveringAlgorithm` class is used to calculate the
        total length covered by the robot along its path. It is updated in the `odomCallback`
        function by calculating the Euclidean distance between the current position of the robot and
        its previous position. This provides a measure of the total distance traveled by the robot
        during its movement. */
        path_length = 0;
        /* The `cumulative_rotation_change` variable in the `ScoreCoveringAlgorithm` class is used to
        keep track of the total change in rotation that the robot has undergone during its movement.
        It is calculated in the `odomCallback` function by comparing the angular velocities of the
        current and previous odometry messages. This provides a measure of how much the robot has
        rotated over time, which can be useful for analyzing its movement patterns and behaviors. */
        cumulative_rotation_change = 0;
        /* The `velocity_smoothness` variable in the `ScoreCoveringAlgorithm` class is used to
        calculate the smoothness of the robot's velocity changes over time. It is updated in the
        `imuCallback` function by calculating the difference in velocity between the current and
        previous IMU messages and dividing it by the time difference between the messages. This
        provides a measure of how smoothly the robot's velocity is changing during its movement. */
        velocity_smoothness = 0;

        /* The `rotation_smoothness` variable in the `ScoreCoveringAlgorithm` class is used to
        calculate the smoothness of the robot's rotation changes over time. It is updated in the
        `imuCallback` function by calculating the difference in angular velocity between the current
        and previous IMU messages and dividing it by the time difference between the messages. This
        provides a measure of how smoothly the robot's rotation is changing during its movement. */
        rotation_smoothness = 0;

        /* The `oscillations_percentage` variable in the `ScoreCoveringAlgorithm` class is used to
        calculate the percentage of time the robot spends in an oscillatory motion state. It is
        updated in the `imuCallback` function by calculating the time duration when the linear and
        angular velocities of the robot are below a certain threshold value (`V_OSC`). If the
        velocities are below this threshold, it indicates that the robot is oscillating or not
        moving significantly in a particular direction. */
        oscillations_percentage = 0;
        /* The `inplace_rotation_percentage` variable in the `ScoreCoveringAlgorithm` class is used to
        calculate the percentage of time the robot spends in an in-place rotation state. It is
        updated in the `imuCallback` function by calculating the time duration when the robot is
        rotating in place, meaning its angular velocity is above a certain threshold value
        (`V_OSC`). If the angular velocity is above this threshold, it indicates that the robot is
        rotating in place rather than moving in a particular direction. The percentage of time spent
        in this in-place rotation state can provide insights into the robot's behavior and movement
        patterns. */
        inplace_rotation_percentage = 0;

        
        v_x_act = 0;
        v_y_act = 0;

        time_start = 0;
        time_end = 0;
    }
    ~ScoreCoveringAlgorithm()
    {
        RCLCPP_INFO(this->get_logger(), "Path length: '%f'", path_length);
        RCLCPP_INFO(this->get_logger(), "Cumulative rotation change: '%f'", cumulative_rotation_change);
        RCLCPP_INFO(this->get_logger(), "Velocity smoothness: '%f'", velocity_smoothness/(iter_imu-1));
        RCLCPP_INFO(this->get_logger(), "Rotation smoothness: '%f'", rotation_smoothness/(iter_imu-1));

        if (time_end - time_start > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Oscillations percentage: '%f'", oscillations_percentage * (100)/(time_end - time_start));
            RCLCPP_INFO(this->get_logger(), "Inplace rotation percentage: '%f'", inplace_rotation_percentage * (100)/(time_end - time_start));
        }

    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Reading velocity float: '%f'", msg -> angular_velocity.x);

        if (iter_imu > 0)
        {
            calcActualValue(msg, prev_imu_msg_);
            velocity_smoothness += calcVelocitySmoothness(msg, prev_imu_msg_);
            rotation_smoothness += calcRotationSmoothness(msg, prev_imu_msg_);
            oscillations_percentage += calcOscillationsTime(msg, prev_imu_msg_);
            inplace_rotation_percentage += calcInPlaceRotationTime(msg, prev_imu_msg_);
            time_end = double(msg -> header.stamp.sec);
        }
        else
        {
            time_start = double(msg -> header.stamp.sec);
        }

        prev_imu_msg_ = msg;
        v_x_prev = v_x_act;
        v_y_prev = v_y_act;
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
        //RCLCPP_INFO(this->get_logger(), "Actual path_length: '%f'",path_length);
        
        drawRobotPoseOnMap.drawPose(msg -> pose.pose.position.x, msg -> pose.pose.position.y);

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

    double calcVelocitySmoothness(  const sensor_msgs::msg::Imu::SharedPtr msg,
                                    const sensor_msgs::msg::Imu::SharedPtr prev_msg) const
    {
        double numerator = double(msg -> header.stamp.sec) - double(prev_msg -> header.stamp.sec);

        if (numerator > 0)
        {
            double denominator = sqrt(  pow(v_x_act - v_x_prev, 2) + 
                                        pow(v_y_act - v_y_prev, 2));
            return denominator/numerator;
        }
        else
            return 0;
    }

    void calcActualValue(   const sensor_msgs::msg::Imu::SharedPtr msg,
                            const sensor_msgs::msg::Imu::SharedPtr prev_msg)
    {
        double time_sec = double(msg -> header.stamp.sec) - double(prev_msg -> header.stamp.sec);
        v_x_act = v_x_act + (msg -> linear_acceleration.x) * time_sec;
        v_y_act = v_y_act + (msg -> linear_acceleration.y) * time_sec;
    }

    double calcRotationSmoothness(  const sensor_msgs::msg::Imu::SharedPtr msg,
                                    const sensor_msgs::msg::Imu::SharedPtr prev_msg) const
    {
        double numerator = double(msg -> header.stamp.sec) - double(prev_msg -> header.stamp.sec);

        if (numerator > 0)
        {
            double denominator = abs(msg -> angular_velocity.z - prev_msg -> angular_velocity.z);
            return denominator/numerator;
        }
        else
            return 0;
    }

    double calcOscillationsTime( const sensor_msgs::msg::Imu::SharedPtr msg,
                                const sensor_msgs::msg::Imu::SharedPtr prev_msg) const
    {
        double V_OSC = 0.01;
        double v_lin = sqrt(pow(v_x_act, 2) + 
                            pow(v_y_act, 2));
        double v_z = abs(msg -> angular_velocity.z);

        if (v_lin < V_OSC && v_x_act < V_OSC && v_y_act < V_OSC && v_z < V_OSC)
        {
            return double(msg -> header.stamp.sec) - double(prev_msg -> header.stamp.sec);
        }
        else
            return 0;
    }

    double calcInPlaceRotationTime( const sensor_msgs::msg::Imu::SharedPtr msg,
                                    const sensor_msgs::msg::Imu::SharedPtr prev_msg) const
    {
        double V_OSC = 0.01;
        // double v_x = abs(msg -> angular_velocity.x);
        // double v_y = abs(msg -> angular_velocity.y);

        if (msg -> angular_velocity.z > V_OSC )
        {
            return double(msg -> header.stamp.sec) - double(prev_msg -> header.stamp.sec);
        }
        else
            return 0;
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
    // void main()
    // {
        
    // }
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
    double time_start;
    double time_end;

    double v_x_act;
    double v_y_act;
    double v_x_prev;
    double v_y_prev;

    double path_length;
    double cumulative_rotation_change;
    double velocity_smoothness;
    double rotation_smoothness;
    double oscillations_percentage;
    double inplace_rotation_percentage;

    DrawRobotPoseOnMap drawRobotPoseOnMap;
};