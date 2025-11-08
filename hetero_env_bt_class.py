#!/usr/bin/env python
import time
import py_trees as pt  # type: ignore
import threading
import asyncio
from collections.abc import Callable
import pybullet as p  # type: ignore
from task_planning_tools.behavior_tree import (
    ConditionBehavior,
    CommandBehavior,
    BehaviorTreePlan,
    merged_loop,
    are_all_trees_done,
)

from pybullet_tools.pr2_utils import (
    DRAKE_PR2_URDF,
    PR2_GROUPS,
    open_arm,
    close_until_collision,
    get_gripper_link,
    get_gripper_joints,
    get_disabled_collisions,
    COMPACT_LEFT_ARM,
    rightarm_from_leftarm,
    close_arm,
)
from pybullet_tools.utils import (
    connect,
    disconnect,
    add_data_path,
    load_model,
    load_pybullet,
    set_pose,
    assign_link_colors,
    plan_joint_motion,
    get_bodies_in_region,
    set_joint_positions,
    get_pose,
    get_link_pose,
    multiply,
    Pose,
    stable_z,
    get_joint_positions,
    quat_from_euler,
    Euler,
    PI,
    invert,
    HideOutput,
    LockRenderer,
    joints_from_names,
    wait_if_gui,
    add_fixed_constraint,
    remove_constraint,
    joint_from_name,
    RGBA,
    interpolate,
    set_color,
    link_from_name,
    approximate_as_prism,
    interpolate_poses,
    get_max_limit,
    get_min_limit,
    pairwise_collision,
)
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
from pybullet_tools.ikfast.pr2.ik import get_if_info


TIME_STEP = 1 / 240.0
STEP_SCALING = 2.0
WAIT_SCALING = 8.0
SIM_SCALING = 2.0
NUM_SIM = 40


class VariableSleeper:
    def __init__(self, interval):
        self.interval = interval
        self.last_time = None

    async def __call__(self):
        current_time = time.perf_counter()
        if self.last_time is None:
            elapsed = 0
        else:
            elapsed = current_time - self.last_time

        sleep_time = max(0.0001, self.interval - elapsed)

        await asyncio.sleep(sleep_time)

        self.last_time = time.perf_counter()

    def reset(self):
        self.last_time = None


class Franka:
    def __init__(self, pose):
        self.stable_config = [0, -PI / 4, 0, -3 * PI / 4, 0, PI / 2, PI / 4]
        with LockRenderer(), HideOutput(True):
            self.robot = load_pybullet(FRANKA_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            assign_link_colors(self.robot, max_colors=2, s=0.7, v=1.0)
            self.tool_link = link_from_name(self.robot, "tool_link")
            self.gripper_joints = [
                joint_from_name(self.robot, "panda_finger_joint1"),
                joint_from_name(self.robot, "panda_finger_joint2"),
            ]
            self.info = PANDA_INFO
            self.ik_joints = get_ik_joints(self.robot, self.info, self.tool_link)
            set_joint_positions(self.robot, self.ik_joints, self.stable_config)
            self.stable_pose = get_link_pose(self.robot, self.tool_link)
            self.constraint = None
            self.vsleep = VariableSleeper(TIME_STEP * STEP_SCALING)

    async def move_to_pose(self, target_pose):
        self.vsleep.reset()
        tool_pose = get_link_pose(self.robot, self.tool_link)
        pose_path = interpolate_poses(
            tool_pose, target_pose, pos_step_size=0.012, spacing="cubic"
        )
        for pose in pose_path:
            conf = next(
                either_inverse_kinematics(
                    self.robot,
                    self.info,
                    self.tool_link,
                    pose,
                    use_pybullet=False,
                    max_distance=1.2,
                    max_time=0.5,
                    max_candidates=100,
                    verbose=False,
                ),
                None,
            )
            if conf is None:
                print("Unable to find IK solution for Franka.")
                return
            set_joint_positions(self.robot, self.ik_joints, conf)
            await self.vsleep()

    async def open_gripper(self):
        open_conf = [get_max_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, open_conf)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)

    async def close_gripper(self):
        close_conf = [get_min_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, close_conf)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)

    async def grasp_gripper(self, obj):
        if self.constraint is None:
            close_until_collision(self.robot, self.gripper_joints, bodies=[obj])
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            self.constraint = add_fixed_constraint(obj, self.robot, self.tool_link)
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
        else:
            print("Franka: Gripper not free")

    async def release_gripper(self, obj):
        if self.constraint is not None:
            placed = get_pose(obj)
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            remove_constraint(self.constraint)
            set_pose(obj, placed)
            self.constraint = None
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            set_pose(obj, placed)
            await self.open_gripper()
            set_pose(obj, placed)
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            set_pose(obj, placed)
        else:
            print("Franka: Gripper is empty")

    async def reset_arm(self, num_steps=50, close_grip=True):
        self.vsleep.reset()
        current_conf = get_joint_positions(self.robot, self.ik_joints)
        joint_path = interpolate(
            current_conf, self.stable_config, num_steps=num_steps, spacing="cubic"
        )
        for conf in joint_path:
            set_joint_positions(self.robot, self.ik_joints, conf)
            await self.vsleep()
        if close_grip is True:
            await self.close_gripper()

    def get_grasp_pose(self, obj):
        body_pose = get_pose(obj)
        center, (w, ln, height) = approximate_as_prism(obj, body_pose=body_pose)
        pick_pose = multiply(
            (center, body_pose[1]),
            Pose(point=[0, 0, 0.5 * height - 0.02]),
            Pose(euler=[0.0, PI, 0.0]),
            Pose(euler=[0.0, 0.0, PI / 2]),
        )
        return pick_pose

    def get_lift_pose(self, grasp_pose):
        return (grasp_pose[0][0], grasp_pose[0][1], self.stable_pose[0][2]), grasp_pose[
            1
        ]

    def get_place_pose(self, obj, place_mark, place_surface):
        temp_z = stable_z(obj, place_surface)
        tool_pose = get_link_pose(self.robot, self.tool_link)
        place_mark = tuple(place_mark[:2]) + (temp_z,)
        body_pose = get_pose(obj)
        grasp = multiply(invert(body_pose), tool_pose)
        body_quat = [0, 0, 0, 1]
        body_pose2 = (place_mark, body_quat)
        with LockRenderer():
            print("[Franka] Paused Simulation for Planning Place Pose")
            while True:
                set_pose(obj, body_pose2)
                if pairwise_collision(obj, place_surface):
                    temp_z += 0.004
                    place_mark = tuple(place_mark[:2]) + (temp_z,)
                    body_pose2 = (place_mark, body_quat)
                    set_pose(obj, body_pose)
                    break
                else:
                    temp_z -= 0.001
                    place_mark = tuple(place_mark[:2]) + (temp_z,)
                    body_pose2 = (place_mark, body_quat)
            print("[Franka] Resumed")
        center, (w, length, height) = approximate_as_prism(obj, body_pose=body_pose2)
        pick_pose = multiply(
            (center, body_pose2[1]), Pose(euler=[0.0, 0.0, PI / 2]), grasp
        )
        return pick_pose

    async def pick_up(self, obj):
        await self.open_gripper()
        franka_pick_pose = self.get_grasp_pose(obj)
        franka_lift_pose = self.get_lift_pose(franka_pick_pose)
        await self.move_to_pose(franka_lift_pose)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)
        await self.move_to_pose(franka_pick_pose)
        await self.grasp_gripper(obj)
        await self.move_to_pose(franka_lift_pose)

    async def place(self, obj, location, surface):
        franka_place_pose = self.get_place_pose(obj, location, surface)
        franka_lift_pose = self.get_lift_pose(franka_place_pose)
        await self.move_to_pose(franka_place_pose)
        await self.release_gripper(obj)
        await self.move_to_pose(franka_lift_pose)


class PR2:
    def __init__(self, pose, planning_arm="right"):
        with LockRenderer(), HideOutput(True):
            self.robot = load_model(DRAKE_PR2_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            set_joint_positions(
                self.robot,
                joints_from_names(self.robot, PR2_GROUPS["left_arm"]),
                COMPACT_LEFT_ARM,
            )
            set_joint_positions(
                self.robot,
                joints_from_names(self.robot, PR2_GROUPS["right_arm"]),
                rightarm_from_leftarm(COMPACT_LEFT_ARM),
            )
            self.select_arm(planning_arm)
            self.constraint = None
            self.gripper_joints = get_gripper_joints(self.robot, self.arm)
            self.vsleep = VariableSleeper(TIME_STEP * STEP_SCALING)

    def select_arm(self, planning_arm):
        self.arm = planning_arm
        self.tool_link = get_gripper_link(self.robot, self.arm)
        self.arm_info = get_if_info(self.arm)
        self.ik_joints = get_ik_joints(self.robot, self.arm_info, self.tool_link)
        self.gripper_joints = get_gripper_joints(self.robot, self.arm)

    async def plan_base_motion(self, goal_conf, obstacles=[]):
        self.vsleep.reset()
        grip_conf = get_joint_positions(self.robot, self.gripper_joints)
        arm_conf = get_joint_positions(self.robot, self.ik_joints)
        disabled = get_disabled_collisions(self.robot)
        base_joints = [joint_from_name(self.robot, name) for name in PR2_GROUPS["base"]]
        with LockRenderer():
            print("[PR2] Paused Simulation for Planning Base Motion")
            base_path = plan_joint_motion(
                self.robot,
                base_joints[:2],
                goal_conf[:2],
                obstacles=obstacles,
                disabled_collisions=disabled,
            )
            print("[PR2] Resumed")
        if base_path is None:
            print("PR2: base path not found")
            return
        for q in base_path:
            set_joint_positions(self.robot, base_joints[:2], q)
            set_joint_positions(self.robot, self.gripper_joints, grip_conf)
            set_joint_positions(self.robot, self.ik_joints, arm_conf)
            await self.vsleep()

    async def arm_motion(self, target_pose):
        self.vsleep.reset()
        tool_pose = get_link_pose(self.robot, self.tool_link)
        pose_path = interpolate_poses(
            tool_pose, target_pose, pos_step_size=0.015, spacing="cubic"
        )
        for pose in pose_path:
            conf = next(
                either_inverse_kinematics(
                    self.robot,
                    self.arm_info,
                    self.tool_link,
                    pose,
                    fixed_joints=[self.ik_joints[0]],
                    use_pybullet=False,
                    max_distance=1.2,
                    max_time=0.3,
                    max_candidates=100,
                    verbose=False,
                ),
                None,
            )
            if conf is None:
                print("PR2: IK not found")
                return
            set_joint_positions(self.robot, self.ik_joints, conf)
            await self.vsleep()

    async def open_gripper(self):
        open_arm(self.robot, self.arm)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)

    async def close_gripper(self):
        close_arm(self.robot, self.arm)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)

    async def grasp_gripper(self, obj):
        if self.constraint is None:
            close_until_collision(self.robot, self.gripper_joints, bodies=[obj])
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            self.constraint = add_fixed_constraint(
                obj, self.robot, get_gripper_link(self.robot, self.arm)
            )
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
        else:
            print("PR2: Gripper not free")

    async def release_gripper(self, obj):
        if self.constraint is not None:
            placed = get_pose(obj)
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            remove_constraint(self.constraint)
            set_pose(obj, placed)
            self.constraint = None
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            set_pose(obj, placed)
            await self.open_gripper()
            set_pose(obj, placed)
            await asyncio.sleep(TIME_STEP * WAIT_SCALING)
            set_pose(obj, placed)

        else:
            print("PR2: Gripper is empty")

    async def reset_arm(self, num_steps=25):
        self.vsleep.reset()
        goal_conf = (
            COMPACT_LEFT_ARM
            if self.arm == "left"
            else rightarm_from_leftarm(COMPACT_LEFT_ARM)
        )
        current_conf = get_joint_positions(self.robot, self.ik_joints[1:])
        joint_path = interpolate(
            current_conf, goal_conf, num_steps=num_steps, spacing="cubic"
        )
        for conf in joint_path:
            set_joint_positions(self.robot, self.ik_joints[1:], conf)
            await self.vsleep()
        await self.close_gripper()

    def get_grasp_pose(self, obj):
        body_pose = get_pose(obj)
        center, (w, length, height) = approximate_as_prism(obj, body_pose=body_pose)
        pick_pose = multiply(
            (center, body_pose[1]),
            Pose(point=[0.045 - 0.5 * length, 0.0, 0.5 * height - 0.02]),
        )
        return pick_pose

    def grasp_approach_base(self, place_mark):
        base_pose = get_pose(self.robot)
        offset = -0.2 if self.arm == "left" else 0.2
        return tuple(
            map(
                lambda x, y: x + y,
                (-0.85, offset, 0.0),
                tuple(place_mark[:2]) + (base_pose[0][2],),
            )
        )

    def get_lift_pose(self, grasp_pose):
        return multiply(grasp_pose, Pose(point=[-0.02, 0.0, 0.14]))

    def get_place_pose(self, obj, place_mark, place_surface):
        temp_z = stable_z(obj, place_surface)
        tool_pose = get_link_pose(self.robot, self.tool_link)
        place_mark = tuple(place_mark[:2]) + (temp_z,)
        body_pose = get_pose(obj)
        grasp = multiply(invert(body_pose), tool_pose)
        body_quat = [0, 0, 0, 1]
        body_pose2 = (place_mark, body_quat)
        with LockRenderer():
            print("[PR2] Paused Simulation for Planning Place Pose")
            while True:
                set_pose(obj, body_pose2)
                if pairwise_collision(obj, place_surface):
                    temp_z += 0.004
                    place_mark = tuple(place_mark[:2]) + (temp_z,)
                    body_pose2 = (place_mark, body_quat)
                    set_pose(obj, body_pose)
                    break
                else:
                    temp_z -= 0.001
                    place_mark = tuple(place_mark[:2]) + (temp_z,)
                    body_pose2 = (place_mark, body_quat)
            print("[PR2] Resumed")
        center, (w, length, height) = approximate_as_prism(obj, body_pose=body_pose2)
        pick_pose = multiply((center, body_pose2[1]), grasp)
        return pick_pose

    async def pick_up(self, obj):
        grasp_pose = self.get_grasp_pose(obj)
        lift_pose = self.get_lift_pose(grasp_pose)
        await self.open_gripper()
        await self.arm_motion(lift_pose)
        await asyncio.sleep(TIME_STEP * WAIT_SCALING)
        await self.arm_motion(grasp_pose)
        await self.grasp_gripper(obj)
        await self.arm_motion(lift_pose)

    async def place(self, obj, location, surface):
        place_pose = self.get_place_pose(obj, location, surface)
        lift_pose = self.get_lift_pose(place_pose)
        await self.arm_motion(place_pose)
        await self.release_gripper(obj)
        await self.arm_motion(lift_pose)

    async def move_base_to_location(self, location, obstacles=[]):
        goal = self.grasp_approach_base(location)
        await self.plan_base_motion(goal, obstacles=obstacles)


class Env:
    def __init__(self, use_gui=True):
        connect(use_gui=use_gui)
        add_data_path()
        self._setup_scene()

    def _setup_scene(self):
        self.plane = p.loadURDF("plane.urdf")

        self.franka_pose = Pose(point=[1.8, 3.5, 0.625])
        self.pr2_pose = Pose()
        table1_pose = ([2.5, 1.2, 0.0], quat_from_euler(Euler(yaw=PI / 2)))
        table2_pose = ([2.0, 3.0, 0.0], quat_from_euler(Euler(yaw=PI / 2)))
        self.common_place_location = (1.8, 3.0, 0.625)

        table1 = load_pybullet("models/table_collision/table.urdf", fixed_base=True)
        set_pose(table1, table1_pose)

        table2 = load_pybullet("models/table_collision/table.urdf", fixed_base=True)
        set_pose(table2, table2_pose)

        plate = load_pybullet("models/dinnerware/plate.urdf", fixed_base=True)
        plate_pose = Pose(point=[2.3, 3.5, stable_z(plate, table2)])
        set_pose(plate, plate_pose)

        cup = load_pybullet("models/dinnerware/cup/cup_small.urdf", fixed_base=False)
        cup_pose = Pose(point=[1.8, 2.5, stable_z(cup, table2)])
        set_pose(cup, cup_pose)

        self.franka_place_location, _ = plate_pose

        block1 = load_pybullet(
            "models/drake/objects/block_for_pick_and_place_small.urdf", fixed_base=False
        )
        set_color(block1, RGBA(0.7, 0.7, 0.2, 1.0))
        set_pose(block1, Pose(point=[2.3, 1.4, stable_z(block1, table1)]))
        self.block1 = block1

        block2 = load_pybullet(
            "models/drake/objects/block_for_pick_and_place_small.urdf", fixed_base=False
        )
        set_color(block2, RGBA(0.6, 0.6, 0.6, 1.0))
        set_pose(block2, Pose(point=[2.1, 1.0, stable_z(block2, table1)]))
        self.block2 = block2

        block3 = load_pybullet(
            "models/drake/objects/block_for_pick_and_place_small.urdf", fixed_base=False
        )
        set_color(block3, RGBA(0.1, 0.5, 0.1, 1.0))
        set_pose(block3, Pose(point=[2.2, 0.8, stable_z(block3, table1)]))
        self.block3 = block3

        self.franka = Franka(self.franka_pose)
        self.pr2 = PR2(self.pr2_pose, "left")

        self.obstacles = [table1, table2, self.franka.robot]

        self.cup, self.table1, self.table2, self.plate = cup, table1, table2, plate

    def is_common_place_empty(self):
        x, y, z = self.common_place_location
        lower = [x - 0.05, y - 0.05, z - 0.01]
        upper = [x + 0.05, y + 0.05, z + 0.03]
        return len(get_bodies_in_region((lower, upper))) <= 1

    async def run_simulation(self, stop_condition_fn: Callable, *args):
        while not stop_condition_fn(*args):
            for _ in range(NUM_SIM):
                p.stepSimulation()
            await asyncio.sleep(TIME_STEP * SIM_SCALING)


class FrankaTree(BehaviorTreePlan):
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        super().__init__()

    def create_tree(self):
        franka_root = pt.composites.Sequence("FrankaTaskPlan", True)

        franka_task1 = pt.composites.Sequence("FrankaPickPlaceTask1", True)
        franka_task2 = pt.composites.Sequence("FrankaPickPlaceTask2", True)
        franka_task3 = pt.composites.Sequence("FrankaPickPlaceTask3", True)

        f11 = ConditionBehavior(
            "Franka: CheckPlaceOccupied", lambda: not self.env.is_common_place_empty()
        )

        f12 = CommandBehavior(
            "Franka: PickUpObject1",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block1,
        )

        f13 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
            close_grip=False,
        )

        f14 = CommandBehavior(
            "Franka: PlaceObject1",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block1,
            self.env.franka_place_location,
            self.env.plate,
        )

        f15 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        f21 = ConditionBehavior(
            "Franka: CheckPlaceOccupied", lambda: not self.env.is_common_place_empty()
        )

        f22 = CommandBehavior(
            "Franka: PickUpObject2",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block2,
        )

        f23 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
            close_grip=False,
        )

        f24 = CommandBehavior(
            "Franka: PlaceObject2",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block2,
            self.env.franka_place_location,
            self.env.block1,
        )

        f25 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        f31 = ConditionBehavior(
            "Franka: CheckPlaceOccupied", lambda: not self.env.is_common_place_empty()
        )

        f32 = CommandBehavior(
            "Franka: PickUpObject3",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block3,
        )

        f33 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
            close_grip=False,
        )

        f34 = CommandBehavior(
            "Franka: PlaceObject3",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block3,
            self.env.franka_place_location,
            self.env.block2,
        )

        f35 = CommandBehavior(
            "Franka: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        franka_task1.add_children([f11, f12, f13, f14, f15])
        franka_task2.add_children([f21, f22, f23, f24, f25])
        franka_task3.add_children([f31, f32, f33, f34, f35])

        franka_root.add_children([franka_task1, franka_task2, franka_task3])

        return pt.trees.BehaviourTree(franka_root)


class PR2Tree(BehaviorTreePlan):
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        super().__init__()

    def create_tree(self):
        pr2_root = pt.composites.Sequence("PR2TaskPlan", True)

        pr2_task1 = pt.composites.Sequence("PR2PickPlaceTask1", True)
        pr2_task2 = pt.composites.Sequence("PR2PickPlaceTask2", True)
        pr2_task3 = pt.composites.Sequence("PR2PickPlaceTask3", True)

        p11 = CommandBehavior(
            "PR2: MoveToObject1",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            get_pose(self.env.block1)[0],
            obstacles=self.env.obstacles,
        )

        p12 = CommandBehavior(
            "PR2: PickUpObject1",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block1,
        )

        p13 = CommandBehavior(
            "PR2: MoveToPlacement",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            self.env.common_place_location,
            obstacles=self.env.obstacles,
        )

        p14 = ConditionBehavior("PR2: CheckPlaceEmpty", self.env.is_common_place_empty)

        p15 = CommandBehavior(
            "PR2: PlaceObject1",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block1,
            self.env.common_place_location,
            self.env.table2,
        )

        p16 = CommandBehavior(
            "PR2: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        p21 = CommandBehavior(
            "PR2: MoveToObject2",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            get_pose(self.env.block2)[0],
            obstacles=self.env.obstacles,
        )

        p22 = CommandBehavior(
            "PR2: PickUpObject2",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block2,
        )

        p23 = CommandBehavior(
            "PR2: MoveToPlacement",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            self.env.common_place_location,
            obstacles=self.env.obstacles,
        )

        p24 = ConditionBehavior("PR2: CheckPlaceEmpty", self.env.is_common_place_empty)

        p25 = CommandBehavior(
            "PR2: PlaceObject2",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block2,
            self.env.common_place_location,
            self.env.table2,
        )

        p26 = CommandBehavior(
            "PR2: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        p31 = CommandBehavior(
            "PR2: MoveToObject3",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            get_pose(self.env.block3)[0],
            obstacles=self.env.obstacles,
        )

        p32 = CommandBehavior(
            "PR2: PickUpObject3",
            self.command_queue,
            self.status_dict,
            self.robot.pick_up,
            self.env.block3,
        )

        p33 = CommandBehavior(
            "PR2: MoveToPlacement",
            self.command_queue,
            self.status_dict,
            self.robot.move_base_to_location,
            self.env.common_place_location,
            obstacles=self.env.obstacles,
        )

        p34 = ConditionBehavior("PR2: CheckPlaceEmpty", self.env.is_common_place_empty)

        p35 = CommandBehavior(
            "PR2: PlaceObject3",
            self.command_queue,
            self.status_dict,
            self.robot.place,
            self.env.block3,
            self.env.common_place_location,
            self.env.table2,
        )

        p36 = CommandBehavior(
            "PR2: ResetArm",
            self.command_queue,
            self.status_dict,
            self.robot.reset_arm,
        )

        pr2_task1.add_children([p11, p12, p13, p14, p15, p16])
        pr2_task2.add_children([p21, p22, p23, p24, p25, p26])
        pr2_task3.add_children([p31, p32, p33, p34, p35, p36])

        pr2_root.add_children([pr2_task1, pr2_task2, pr2_task3])

        return pt.trees.BehaviourTree(pr2_root)


async def main():
    env = Env(use_gui=True)
    franka = FrankaTree(env, env.franka)
    pr2 = PR2Tree(env, env.pr2)

    bt_thread = threading.Thread(target=merged_loop, args=(franka, pr2), daemon=True)
    rest = TIME_STEP * WAIT_SCALING
    wait_if_gui("Start?")
    print("The Behavior Trees for Individual Robots:")
    pr2.display_tree()
    franka.display_tree()
    start_time = time.perf_counter()

    print("Running BT in background thread...")
    bt_thread.start()

    await asyncio.gather(
        env.run_simulation(are_all_trees_done, franka, pr2),
        franka.execute_tree(rest),
        pr2.execute_tree(rest),
    )

    end_time = time.perf_counter()
    print(f"Execution time: {end_time - start_time:.4f} seconds")
    wait_if_gui("Finish?")
    disconnect()


if __name__ == "__main__":
    asyncio.run(main())
