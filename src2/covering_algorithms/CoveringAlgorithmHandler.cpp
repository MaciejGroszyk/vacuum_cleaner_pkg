#include "CoveringAlgorithmHandler.h"


CoveringAlgorithmHandler::CoveringAlgorithmHandler()
{

}

CoveringAlgorithmHandler::~CoveringAlgorithmHandler()
{

}

void CoveringAlgorithmHandler::start_(std::shared_ptr<CoveringAlgorithm> ca_node)
{
    executor.add_node(ca_node);
    executor.add_node(ca_node -> robot_controller_node);
    executor.add_node(ca_node -> laser_handler_node);
    executor.add_node(ca_node -> odom_handler_node);

    executor.spin();
}

void CoveringAlgorithmHandler::startRandomWalk()
{
    auto random_walk_node = std::make_shared<RandomWalk>();
    start_(random_walk_node);
}

void CoveringAlgorithmHandler::startSpiralWalk()
{
    auto spiral_walk_node = std::make_shared<SpiralWalk>();
    start_(spiral_walk_node);
}

void CoveringAlgorithmHandler::startSnackingWalk()
{

}