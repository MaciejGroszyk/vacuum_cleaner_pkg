
#include <iostream>
#include "RobotControler.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RobotControler>());

    auto rc_node = std::make_shared<RobotControler>();

    bool running = true;
    char ch;
    while(running)
    {
        std::cin.get(ch);
        // TO DO keyboard handler without etner confirmation
        RobotControler::MoveCommands cmd = RobotControler::MoveCommands::STOP;
        if (ch== 'w')
        {
            cmd = RobotControler::MoveCommands::FORWARD;
            rc_node -> move(cmd);
        }
        else if (ch == 's')
        {
            cmd = RobotControler::MoveCommands::STOP;
            rc_node -> move(cmd); 
        }
        else if (ch == 'a')
        {
            rc_node -> move(RobotControler::MoveCommands::LEFT); 
        }
        else if (ch == 'd')
        {
            rc_node -> move(RobotControler::MoveCommands::RIGHT); 
        }
        else if (ch == 'q')
        {
            running = false; 
        }
    }



    rclcpp::shutdown();
    return 0;
}
