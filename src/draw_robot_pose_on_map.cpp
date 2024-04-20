#include <ament_index_cpp/get_package_prefix.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp> 
#include <string>
#include <tuple>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
// def rotate(points, angle):
//     ANGLE = np.deg2rad(angle)
//     c_x, c_y = np.mean(points, axis=0)
//     return np.array(
//         [
//             [
//                 c_x + np.cos(ANGLE) * (px - c_x) - np.sin(ANGLE) * (py - c_x),
//                 c_y + np.sin(ANGLE) * (px - c_y) + np.cos(ANGLE) * (py - c_y)
//             ]
//             for px, py in points
//         ]
//     ).astype(int)


class DrawRobotPoseOnMap
{
private:
    const std::string package_name = "random_world_generator_pkg";
    const std::string img_path = "/data/map_model/current_map_img.png";
    const std::string json_path = "/data/map_model/stl_size.json";

    const float SCALE = 100;
    const std::tuple <float, float> TURTLEBOT3_WAFLE_SIZE{0.306*SCALE, 0.281*SCALE}; // (width, height)
    unsigned long long int iter = 0;

    cv::Mat image;
    float map_width;
    float map_height;

public:
    DrawRobotPoseOnMap()
    {
        readCurrentMapPng();
        readCurrentMapSize();
    }
    ~DrawRobotPoseOnMap()
    {
        saveCurrentMapPng();
        std::cout << iter << std::endl;
    }


    void drawPose(const float &x, const float &y)
    {
        std::vector<cv::Point> rec = getScaledRobotPose(cv::Point(int(x*SCALE), int(y*SCALE)));
        std::cout << "----------------" << std::endl;
        std::cout << y << std::endl;
        std::cout << x << std::endl;
        std::cout << y*SCALE << std::endl;
        std::cout << x*SCALE << std::endl;
        cv::rectangle(image, rec[0], rec[1], cv::Vec3b(255,0,0), -1, cv::LINE_8);
        ++iter;
    }

    std::vector<cv::Point> getScaledRobotPose(const cv::Point &robot_pose) const
    {
        const int r_w = (int)(std::get<0>(TURTLEBOT3_WAFLE_SIZE)/ 2.0);
        const int r_h = (int)(std::get<1>(TURTLEBOT3_WAFLE_SIZE) / 2.0);

        return
        {   cv::Point(robot_pose.x - r_w, robot_pose.y - r_h),
            cv::Point(robot_pose.x + r_w, robot_pose.y + r_h)};
    }

    void readCurrentMapPng()
    {
        const std::string package_path = getPackageSrcPath();
        image = cv::imread(package_path + img_path, cv::IMREAD_UNCHANGED); 
    }

    void readCurrentMapSize()
    {
        const std::string package_path = getPackageSrcPath();

        std::ifstream f(package_path+json_path);
        json data = json::parse(f);

        map_width = data["width"];
        map_height = data["height"];
    }

    void saveCurrentMapPng()
    {
        const std::string package_path = getPackageSrcPath();
        cv::imwrite(package_path + "/data/map_model/current_map_img_with_path.png", image); 
    }

    std::string getPackageSrcPath() const
    {
        std::string package_path = ament_index_cpp::get_package_prefix(package_name);
        return package_path.replace(package_path.find("install"),7, "src");
    }
};

// int main() 
// {
//     DrawRobotPoseOnMap drpom;

    // std::string package_path = ament_index_cpp::get_package_prefix("random_world_generator_pkg");
    // package_path.replace(package_path.find("install"),7, "src");
    // std::string img_path = "/data/map_model/current_map_img.png";
    // cv::Mat image = cv::imread(package_path + img_path,
    //                 cv::IMREAD_UNCHANGED); 

    // // for (int i = 0; i < 20; ++i) {
    // //     for (int j = 0; j < 20; ++j)
    // //     {
    // //         image.at<cv::Vec3b>(20+i,20+j)=cv::Vec3b(0,0,255);
    // //     }
    // // }
    // cv::Point p1(30, 30); 
    // cv::Point p2(255, 255); 
    // cv::rectangle(image, p1, p2, cv::Vec3b(0,0,255), -1, cv::LINE_8);
    // // image.at<cv::Vec3b>(20,20)=cv::Vec3b(0,0,255);
    // cv::imshow("image", image);
    // cv::waitKey(0); 

    // std::cout << image.size();
//     return 0;
// }