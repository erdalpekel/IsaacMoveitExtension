# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass


from omni.isaac.core.utils.types import ArticulationAction
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.surface_gripper._surface_gripper import (
    Surface_Gripper_Properties,
    Surface_Gripper,
)


class ExampleScenario(ScenarioTemplate):
    def __init__(self):
        self._running_scenario = False
        self._time = 0.0  # s

        self.action_graph = None
        self._setup_ros_actiongraph()
        self.init_gripper()
        self.surface_gripper.open()

    def setup_scenario(self, articulation):
        self._running_scenario = True

    def teardown_scenario(self):
        self._time = 0.0
        self._running_scenario = False

        self.action_graph = None

    def update_scenario(self, step: float):
        if not self._running_scenario:
            return

        self.surface_gripper.update()

    def close_gripper(self):
        if not self._running_scenario:
            return
        self.surface_gripper.close()

    def init_gripper(self):
        self._dc = dc.acquire_dynamic_control_interface()
        self.sgp = Surface_Gripper_Properties()
        self.sgp.d6JointPath = "/World/panda/panda_tip/d6FixedJoint"
        self.sgp.parentPath = "/World/panda/panda_tip"
        self.sgp.offset = dc.Transform()  # Offset the transform to the base of the Cone
        self.sgp.offset.p.x = -0.001
        self.sgp.offset.r = [1.0, 0, 0, 0]
        self.sgp.gripThreshold = 0.02
        self.sgp.forceLimit = 1.0e3
        self.sgp.torqueLimit = 1.0e3
        self.sgp.bendAngle = 7.5
        self.sgp.stiffness = 1.0e4
        self.sgp.damping = 1.0e3
        self.sgp.retryClose = True

        self.surface_gripper = Surface_Gripper(self._dc)
        self.surface_gripper.initialize(self.sgp)

    def _setup_ros_actiongraph(self):
        print(f"Create ActionGraph /franka_graph")
        try:
            (action_graph_local, _, _, _) = og.Controller.edit(
                {
                    "graph_path": "/franka_graph",
                    "evaluator_name": "execution",
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        (
                            "SubscribeROSTime",
                            "omni.isaac.ros2_bridge.ROS2SubscribeClock",
                        ),
                        (
                            "PublishTF",
                            "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
                        ),
                        (
                            "PublishRobotJointStates",
                            "omni.isaac.ros2_bridge.ROS2PublishJointState",
                        ),
                        (
                            "SubscribeRobotJointStates",
                            "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                        ),
                        (
                            "RobotArticulationController",
                            "omni.isaac.core_nodes.IsaacArticulationController",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "RobotArticulationController.inputs:robotPath",
                            "/World/panda",
                        ),
                        (
                            "PublishRobotJointStates.inputs:topicName",
                            "/joint_states",
                        ),
                        (
                            "SubscribeRobotJointStates.inputs:topicName",
                            "/joint_commands",
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (
                            "OnPlaybackTick.outputs:tick",
                            "SubscribeROSTime.inputs:execIn",
                        ),
                        (
                            "OnPlaybackTick.outputs:tick",
                            "PublishRobotJointStates.inputs:execIn",
                        ),
                        (
                            "OnPlaybackTick.outputs:tick",
                            "SubscribeRobotJointStates.inputs:execIn",
                        ),
                        (
                            "OnPlaybackTick.outputs:tick",
                            "RobotArticulationController.inputs:execIn",
                        ),
                        (
                            "SubscribeROSTime.outputs:timeStamp",
                            "PublishRobotJointStates.inputs:timeStamp",
                        ),
                        (
                            "SubscribeRobotJointStates.outputs:effortCommand",
                            "RobotArticulationController.inputs:effortCommand",
                        ),
                        (
                            "SubscribeRobotJointStates.outputs:positionCommand",
                            "RobotArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeRobotJointStates.outputs:velocityCommand",
                            "RobotArticulationController.inputs:velocityCommand",
                        ),
                        (
                            "SubscribeRobotJointStates.outputs:jointNames",
                            "RobotArticulationController.inputs:jointNames",
                        ),
                        (
                            "OnPlaybackTick.outputs:tick",
                            "SubscribeROSTime.inputs:execIn",
                        ),
                        (
                            "SubscribeROSTime.outputs:timeStamp",
                            "PublishTF.inputs:timeStamp",
                        ),
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ],
                },
                update_usd=True,
            )
            print(f"ActionGraph: {action_graph_local}")
            self.action_graph = action_graph_local

            set_target_prims(
                primPath="/franka_graph/PublishRobotJointStates",
                targetPrimPaths=["/World/panda"],
            )
            set_target_prims(
                primPath="/franka_graph/RobotArticulationController",
                targetPrimPaths=["/World/panda"],
            )

            set_target_prims(
                primPath="/franka_graph/PublishTF",
                inputName="inputs:parentPrim",
                targetPrimPaths=["/World/panda"],
            )
            set_target_prims(
                primPath="/franka_graph/PublishTF",
                inputName="inputs:targetPrims",
                targetPrimPaths=["/World/panda"],
            )

        except Exception as exception:
            print("Exception ActionGraph")
            print(exception)
