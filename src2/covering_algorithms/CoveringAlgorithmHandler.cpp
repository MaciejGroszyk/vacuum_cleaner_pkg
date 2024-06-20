#include "CoveringAlgorithmHandler.h"


CoveringAlgorithmHandler::CoveringAlgorithmHandler()
{

}

CoveringAlgorithmHandler::~CoveringAlgorithmHandler()
{
    
}

void CoveringAlgorithmHandler::startRandomWalk()
{
    auto rw_node = std::make_shared<RandomWalk>();

    executor.add_node(rw_node);
    executor.add_node(rw_node -> rc_node);
    executor.add_node(rw_node -> lh_node);
    executor.add_node(rw_node -> oh_node);

    executor.spin();

}