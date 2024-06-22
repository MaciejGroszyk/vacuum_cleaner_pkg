
#include <iostream>
#include "robot_handler/RobotController.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RobotController>());

    auto rc_node = std::make_shared<RobotController>();

    bool running = true;
    char ch;
    while(running)
    {
        std::cin.get(ch);
        // TO DO keyboard handler without enter confirmation
        RobotController::MoveCommands cmd = RobotController::MoveCommands::STOP;
        if (ch== 'w')
        {
            cmd = RobotController::MoveCommands::FORWARD;
            rc_node -> move(cmd);
        }
        else if (ch == 's')
        {
            cmd = RobotController::MoveCommands::STOP;
            rc_node -> move(cmd); 
        }
        else if (ch == 'a')
        {
            rc_node -> move(RobotController::MoveCommands::LEFT); 
        }
        else if (ch == 'd')
        {
            rc_node -> move(RobotController::MoveCommands::RIGHT); 
        }
        else if (ch == 'q')
        {
            running = false; 
        }
    }



    rclcpp::shutdown();
    return 0;
}
