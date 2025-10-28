#!/usr/bin/env python
from __future__ import print_function
import pybullet as p

from pybullet_tools.pr2_utils import (
    DRAKE_PR2_URDF, PR2_GROUPS, open_arm, close_until_collision,
    get_gripper_link, get_gripper_joints, get_disabled_collisions,
    COMPACT_LEFT_ARM, rightarm_from_leftarm
)
from pybullet_tools.utils import (
    connect, disconnect, add_data_path, load_model, load_pybullet, set_pose, assign_link_colors,
    plan_joint_motion, set_joint_positions, get_pose, get_link_pose, multiply, Pose, stable_z,
    get_joint_positions, quat_from_euler, Euler, PI, HideOutput, LockRenderer,joints_from_names,
    wait_if_gui, add_fixed_constraint, remove_constraint, joint_from_name, RGBA, interpolate,
    link_from_name, approximate_as_cylinder, interpolate_poses, create_box, get_max_limit, get_min_limit
)
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
from pybullet_tools.ikfast.pr2.ik import get_if_info

PANDA_STABLE_CONFIG = [0, -PI/4, 0, -3*PI/4, 0, PI/2, PI/4]
PANDA_OPEN_GRIPPER = 0.04
PANDA_CLOSE_GRIPPER = 0.

class Franka:
    def __init__(self, pose):
        with LockRenderer(), HideOutput(True):
            self.robot = load_pybullet(FRANKA_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            assign_link_colors(self.robot, max_colors=2, s=0.7, v=1.0)
            self.tool_link = link_from_name(self.robot, 'tool_link')
            self.gripper_joints = [joint_from_name(self.robot, 'panda_finger_joint1'), joint_from_name(self.robot, 'panda_finger_joint2')]
            self.info = PANDA_INFO
            self.ik_joints = get_ik_joints(self.robot, self.info, self.tool_link)
            set_joint_positions(self.robot, self.ik_joints, PANDA_STABLE_CONFIG)
        

    def move_to_pose(self, target_pose):
        tool_link = self.tool_link
        tool_pose = get_link_pose(self.robot, tool_link)
        pose_path = interpolate_poses(tool_pose, target_pose, pos_step_size=0.025)
        for pose in pose_path:
            conf = next(either_inverse_kinematics(
                self.robot, self.info, tool_link, pose,
                use_pybullet=False, max_distance=1.5, max_time=0.5,
                max_candidates=100, verbose=False
            ), None)
            if conf is None:
                print('Unable to find IK solution for Franka.')
                return
            set_joint_positions(self.robot, self.ik_joints, conf)
            for _ in range(200):
                p.stepSimulation()
    
    def open_gripper(self):
        open_conf = [get_max_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, open_conf)
        for _ in range(200):
            p.stepSimulation()

    def close_gripper(self):
        close_conf = [get_min_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, close_conf)
        for _ in range(200):
            p.stepSimulation()
    
    def grasp_gripper(self, bodies=[]):
        close_until_collision(self.robot, self.gripper_joints, bodies=bodies)
        for _ in range(200):
            p.stepSimulation()

    def return_to_stable(self, num_steps):
        current_conf = get_joint_positions(self.robot, self.ik_joints)
        joint_path =  interpolate(current_conf, PANDA_STABLE_CONFIG, num_steps=num_steps)
        for conf in joint_path:
            set_joint_positions(self.robot, self.ik_joints, conf)
            for _ in range(200):
                p.stepSimulation()
        self.close_gripper()
        


class PR2:
    def __init__(self, pose):
        with LockRenderer(), HideOutput(True):
            self.robot = load_model(DRAKE_PR2_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            set_joint_positions(self.robot, joints_from_names(self.robot,PR2_GROUPS['left_arm']), COMPACT_LEFT_ARM)
            set_joint_positions(self.robot, joints_from_names(self.robot,PR2_GROUPS['right_arm']), rightarm_from_leftarm(COMPACT_LEFT_ARM))

    def plan_base_motion(self, goal_conf, obstacles=[]):
        disabled = get_disabled_collisions(self.robot)
        base_joints = [joint_from_name(self.robot, name) for name in PR2_GROUPS['base']]
        with LockRenderer():
            base_path = plan_joint_motion(self.robot, base_joints[:2], goal_conf[:2],
                                      obstacles=obstacles, disabled_collisions=disabled, verbose=False)
        if base_path is None:
            print("PR2: base path not found")
            return
        for q in base_path:
            set_joint_positions(self.robot, base_joints[:2], q)
            for _ in range(100):
                p.stepSimulation()

    def arm_motion(self, arm, target_pose):
        tool_link = get_gripper_link(self.robot, arm)
        tool_pose = get_link_pose(self.robot, tool_link)
        info = get_if_info(arm)
        ik_joints = get_ik_joints(self.robot, info, tool_link)
        pose_path = interpolate_poses(tool_pose, target_pose, pos_step_size=0.025)
        for pose in pose_path:
            conf = next(either_inverse_kinematics(
                self.robot, info, tool_link, pose, fixed_joints=[ik_joints[0]],
                use_pybullet=False, max_distance=1.0, max_time=0.2,
                max_candidates=100, verbose=False
            ), None)
            if conf is None:
                print("PR2: IK not found")
                return
            set_joint_positions(self.robot, ik_joints, conf)
            for _ in range(200):
                p.stepSimulation()
    
    def grasp_gripper(self, arm, bodies=[]):
        gjoints = get_gripper_joints(self.robot, arm)
        close_until_collision(self.robot, gjoints, bodies=bodies)
        for _ in range(200):
            p.stepSimulation()


class Env:
    def __init__(self, use_gui=True):
        connect(use_gui=use_gui)
        add_data_path()
        self._setup_scene()
        self.franka = Franka(self.franka_pose)
        self.pr2 = PR2(self.pr2_pose)

    def _setup_scene(self):
        self.plane = p.loadURDF("plane.urdf")

        self.franka_pose = Pose(point=[1.8, 3.5, 0.625])
        self.pr2_pose = Pose()
        table1_pose = ([1., 0., 0.], quat_from_euler(Euler(yaw=PI / 2)))
        table2_pose = ([2., 3., 0.], quat_from_euler(Euler(yaw=PI / 2)))

        table1 = load_pybullet("models/table_collision/table.urdf", fixed_base=True)
        set_pose(table1, table1_pose)

        table2 = load_pybullet("models/table_collision/table.urdf", fixed_base=True)
        set_pose(table2, table2_pose)

        plate = load_pybullet("models/dinnerware/plate.urdf", fixed_base=True)
        plate_pose = Pose(point=[2.2, 3.5, stable_z(plate, table2)])
        set_pose(plate, plate_pose)

        cup = load_pybullet("models/dinnerware/cup/cup_small.urdf", fixed_base=False)
        cup_pose = Pose(point=[1.8, 2.5, stable_z(cup, table2)])
        set_pose(cup, cup_pose)

        block1 = create_box(0.05, 0.05, 0.05, mass=0.1, color=RGBA(0.7,0.7,0.2,1.))
        set_pose(block1, Pose(point=[0.9,-0.3,stable_z(block1, table1)]))
        self.block1 = block1

        block2 = create_box(0.05, 0.05, 0.05, mass=0.1, color=RGBA(0.6,0.6,0.6,1.))
        set_pose(block2, Pose(point=[0.9,0., stable_z(block2, table1)]))
        self.block2 = block2

        block3 = create_box(0.05, 0.05, 0.05, mass=0.1, color=RGBA(0.7,0.2,0.7,1.))
        set_pose(block3, Pose(point=[0.9,0.3,stable_z(block1, table1)]))
        self.block3 = block3

        self.cup, self.table1, self.table2, self.plate = cup, table1, table2, plate

    def execute_task(self):
        pr2, franka, cup, plate = self.pr2, self.franka, self.cup, self.plate

        b_goal = (0.9, 2.7, 0.)
        b_goal2 = (0.8, 3.4, 0.)
        arm = 'right'

        pr2.plan_base_motion(b_goal, obstacles=[self.table1, self.table2, self.plane, cup])
        open_arm(pr2.robot, arm)
        center, (diameter, height) = approximate_as_cylinder(cup)
        grasp_pose = multiply(get_pose(cup), Pose(point=[-0.1 * diameter, 0., 0.5 * height]))
        pr2.arm_motion(arm, grasp_pose)
        pr2.grasp_gripper(arm, bodies=[cup])
        c1 = add_fixed_constraint(cup, pr2.robot, get_gripper_link(pr2.robot, arm))

        for _ in range(500):
            p.stepSimulation()

        lift_pose = multiply(grasp_pose, Pose(point=[-0.1 * diameter, 0, height * 2.]))
        pr2.arm_motion(arm, lift_pose)
        pr2.plan_base_motion(b_goal2, obstacles=[self.table1, self.table2, self.plane, cup, self.franka.robot])

        franka.open_gripper()
        franka_pick_pose = multiply(get_pose(cup), Pose(point=[0, 0, 0.5 * height]), Pose(euler=[0., PI, 0.]), Pose(euler=[0., 0., PI/2]))
        franka.move_to_pose(franka_pick_pose)
        c2 = add_fixed_constraint(cup, franka.robot, franka.tool_link)
        remove_constraint(c1)
        open_arm(pr2.robot, arm)
        franka.grasp_gripper(bodies=[cup])
        #franka.set_gripper(0.018)

        for _ in range(500):
            p.stepSimulation()

        franka_pre_pose = multiply(franka_pick_pose,Pose(point=[0, 0, -6.*height]))
        franka.move_to_pose(franka_pre_pose)

        place_pose = multiply(get_pose(plate), Pose(point=[0, 0, 1. * height]), Pose(euler=[0., PI, 0.]))
        franka.move_to_pose(place_pose)
        franka.open_gripper()

        remove_constraint(c2)
        for _ in range(500):
            p.stepSimulation()

        franka.return_to_stable(num_steps=20)

        print("Task execution complete.")



def main():
    env = Env(use_gui=True)
    
    for _ in range(500):
        p.stepSimulation()

    wait_if_gui('Start?')
    
    env.execute_task()

    for _ in range(500):
        p.stepSimulation()

    wait_if_gui('Finish?')
    disconnect()


if __name__ == '__main__':
    main()
