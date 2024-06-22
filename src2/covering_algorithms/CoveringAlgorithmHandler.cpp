#include "CoveringAlgorithmHandler.h"


CoveringAlgorithmHandler::CoveringAlgorithmHandler()
{

}

CoveringAlgorithmHandler::~CoveringAlgorithmHandler()
{
    
}

void CoveringAlgorithmHandler::startRandomWalk()
{
    auto random_walk_node = std::make_shared<RandomWalk>();

    executor.add_node(random_walk_node);
    executor.add_node(random_walk_node -> rc_node);
    executor.add_node(random_walk_node -> lh_node);
    executor.add_node(random_walk_node -> oh_node);

    executor.spin();

}

void CoveringAlgorithmHandler::startSpiralWalk()
{
    auto spiral_walk_node = std::make_shared<SpiralWalk>();

    executor.add_node(spiral_walk_node);
    executor.add_node(spiral_walk_node -> rc_node);
    executor.add_node(spiral_walk_node -> robot_controller_node);
    executor.add_node(spiral_walk_node -> laser_handler_node);
    executor.add_node(spiral_walk_node -> odom_handler_node);

    executor.spin();

}