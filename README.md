# Simulation system for the development of autonomous
ROS 2 - based project implementing simple covering area with turtlebot3 as vacuum cleaner.

Simulation system for autonomous mobile robot navigation evaluation, which enables to create dynamic simulation environment with random rooms and emerging obstacles. System is described in Domain Specification Language SPSysML ([https://arxiv.org/abs/2303.09565](https://arxiv.org/abs/2303.09565)) , which rates system simulation-physical integrality. The thesis proposes indicators used to examinate navigation algorithms. The work is a proposal of a standard for quality indicators used when evaluating navigation algorithms of mobile robots

## Requirement specification
![system_requirements](docx/diagrams/system_requirements.png) 
![Requirements](docx/diagrams/Requirements.png) 

![example_use_case](docx/diagrams/example_use_case.png)

## Project structure
![main](docx/diagrams/main.png)

## World synchronization
![simulated_world](docx/diagrams/simulated_world.png)

Flow diagram
![ibd_general_world generator](docx/diagrams/ibd_general_world_generator.png)


Load visualization
![Load_World From_Data](docx/diagrams/Load_World_From_Data.png)

Load visualization example
![current_map_img](docx/img/current_map_img.jpg)
![current_map_model_3D](docx/img/current_map_model_3D.png)

Generating the real world to simulation
![Generate_Real_World](docx/diagrams/Generate_Real_World.png)

Generating the real world to simulation example

![lab_map](docx/img/lab_map.png) 
![lab_map_without_noise](docx/img/lab_map_without_noise.png) 
![lab_map_gazebo](docx/img/lab_map_gazebo.png) 

Generate random visualization

![Random_World_Generator](docx/diagrams/Random_World_Generator.png) 

![sq_Random_World_Generator](docx/diagrams/sq_Random_World_Generator.png)

Generate random visualization example
![random_world_generator](docx/img/random_world_generator.jpg) 


Spawninng obstacles
![sq_spawn_obstacle](docx/diagrams/sq_spawn_obstacle.png) 



## DT/PT decomposition – Mirroring Agent Groups specification

Simulation agent
![sim_realization](docx/diagrams/sim_realization.png)

Physical agent
![phy_realization](docx/diagrams/phy_realization.png)

## Agent decomposition
Simulation agent decomposition

![simulated_sega_ibd](docx/diagrams/simulated_sega_ibd.png)

Physical agent decomposition

![physical_sega_ibd](docx/diagrams/physical_sega_ibd.png)

## Structure evaluation

**Controller integrity factor** IFF = 1. The higher the IIF value, the more software components are shared between the simulation and physical embodiments. Maximum IIF value is 1 - all abstract hardware parts of the system are common.

**Driver generalisation factor** DGF = 1. The higher the value, the easier it is to integrate physical and simulated hardware in the future.

**Digital Twin coverage** DTC = 1. All physical parts have digital twins.

**Mirror integrity factor** MIF<sub>Robot</sub> = 1. Parts of the system between the physical and digital twins that make up the robot are shared.