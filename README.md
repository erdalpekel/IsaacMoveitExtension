# Omniverse Isaac MoveIt! Samples 

In this repository, we publish the code necessary for running the sample Omniverse Isaac extension that I introduced in my blog in a series of blog posts.

## Changelog

[Control Panda with MoveIt! in Omniverse Isaac](https://erdalpekel.de/?p=1170)

In this blog post, we will import an existing robotic arm model into NVIDIA’s Omniverse Isaac simulation environment and create a ROS package and an Isaac extension for controlling the arm with MoveIt!. MoveIt! will control to the robotic arm inside Isaac via the ROS Bridge.

![Control Panda with MoveIt! in Omniverse Isaac](assets/execute-trajectory.png?raw=true "Control Panda with MoveIt! in Omniverse Isaac")

[Add basic suction gripper to Panda in Omniverse Isaac](https://erdalpekel.de/?p=1256)

In this blog post, we will add a suction gripper to the Panda robotic arm in NVIDIA’s Omniverse Isaac simulation environment. In our latest contribution, we added the Panda robotic arm to Isaac Sim and controlled it with MoveIt! via ros_bridge. We will continue this series by modifying the arm’s model and by adding the suction gripper functionality. The resulting robotic arm model will be able to grip rigid bodies with its gripper in the simulation. 

![Add basic suction gripper to Panda in Omniverse Isaac](assets/pick-with-suction-gripper.png?raw=true "Add basic suction gripper to Panda in Omniverse Isaac")

[Pick and Place with Semantic Instance Segmentation in Omniverse Isaac](https://erdalpekel.de/?p=1293)

In this blog post, we will pick and place simple cubes onto a conveyor belt with the Panda robotic arm in NVIDIA’s Omniverse Isaac simulation environment. We will detect the cubes with an RGB camera and Isaac Sim’s built-in semantic instance segmentation algorithm. In our latest contributions, we added the Panda robotic arm to Isaac Sim and controlled it with MoveIt! via ros_bridge. We also attached a custom modeled suction gripper part to the arm. In the resulting simulation, the robotic arm is able to pick a cube that the camera has detected and place it on the conveyor belt for carrying.

![Pick and Place with Semantic Instance Segmentation in Omniverse Isaac](assets/pick-place-semantic-segmentation.png?raw=true "Pick and Place with Semantic Instance Segmentation in Omniverse Isaac")