class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass


import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

import omni.physics.tensors as physics
from omni.isaac.surface_gripper._surface_gripper import (
    Surface_Gripper_Properties,
    Surface_Gripper,
)

from omni.isaac.sensor import Camera
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim

from omni.isaac.core import utils

utils.extensions.enable_extension("omni.isaac.ros2_bridge")
import rclpy
from .ros_interface import ROSInterface


class ExampleScenario(ScenarioTemplate):
    def __init__(self):

        if not rclpy.ok():
            rclpy.init()

        self._running_scenario = False
        self._time = 0.0  # s

        self.ros_interface = ROSInterface()

        self.action_graph = None
        self._setup_ros_actiongraph()
        self.init_gripper()
        self.surface_gripper.open()

        self.cube_prim_pose = None

    def setup_scenario(self):
        self._running_scenario = True

        self.camera_prim_path = "/World/Group/Camera"
        self.scene = World.instance().scene

        self.camera = Camera(prim_path=self.camera_prim_path)
        self.camera.initialize()
        self.camera.add_instance_segmentation_to_frame()
        frame = self.camera.get_current_frame()
        print(f"Camera current frame: {frame}")

    def teardown_scenario(self):
        self._time = 0.0
        self._running_scenario = False

        self.action_graph = None

    def update_scenario(self, step: float):
        if not self._running_scenario:
            return

        self.surface_gripper.update()

        frame = self.camera.get_current_frame()
        if "instance_segmentation" in frame:
            instance_segmentation = frame["instance_segmentation"]
            cubes = {}
            if instance_segmentation:
                id_to_labels = instance_segmentation["info"]["idToLabels"]
                for id, semantic_label in instance_segmentation["info"][
                    "idToSemantics"
                ].items():
                    if "cube" in semantic_label["class"]:
                        cubes[id_to_labels[id]] = int(id)

                for cube_key in cubes:
                    prim_path = cube_key
                    if not self.scene.object_exists(prim_path):
                        self.scene.add(XFormPrim(prim_path=prim_path, name=prim_path))
                    cube_prim_object = self.scene.get_object(prim_path)
                    self.cube_prim_pose = cube_prim_object.get_world_pose()
                    # print(f"cube {cubes[cube_key]} world transform: {cube_prim_pose[0]}")

    def move_to_conveyor(self):
        if not self._running_scenario:
            return

        cube_dropoff_position = (
            XFormPrim(prim_path="/World/cube_dropoff_position")
            .get_world_pose()[0]
            .tolist()
        )
        print(f"cube dropoff position: {cube_dropoff_position}")
        if cube_dropoff_position:
            print(f"cube dropoff position: {cube_dropoff_position}")
            self.ros_interface.send_goal(cube_dropoff_position)

    def move_to_cube(self):
        if not self._running_scenario:
            return

        if self.cube_prim_pose:
            print(f"cube prim pose: {self.cube_prim_pose[0]}")
            self.ros_interface.send_goal(self.cube_prim_pose[0])

    def grasp(self):
        if not self._running_scenario:
            return
        self.surface_gripper.close()

    def open_gripper(self):
        if not self._running_scenario:
            return
        self.surface_gripper.open()

    def init_gripper(self):
        self.sgp = Surface_Gripper_Properties()
        self.sgp.d6JointPath = "/World/panda/panda_tip/d6FixedJoint"
        self.sgp.parentPath = "/World/panda/panda_tip"
        self.sgp.offset = physics.Transform()
        self.sgp.offset.p.x = -0.001
        self.sgp.offset.r = [1.0, 0, 0, 0]
        self.sgp.gripThreshold = 0.02
        self.sgp.forceLimit = 1.0e3
        self.sgp.torqueLimit = 1.0e3
        self.sgp.bendAngle = 7.5
        self.sgp.stiffness = 1.0e4
        self.sgp.damping = 1.0e3
        self.sgp.retryClose = True

        self.surface_gripper = Surface_Gripper()
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
                targetPrimPaths=["/World"],
            )
            set_target_prims(
                primPath="/franka_graph/PublishTF",
                inputName="inputs:targetPrims",
                targetPrimPaths=["/World/panda"],
            )

        except Exception as exception:
            print("Exception ActionGraph")
            print(exception)
