#!/usr/bin/env python
from __future__ import print_function
import pybullet as p

from pybullet_tools.pr2_utils import (
    DRAKE_PR2_URDF, PR2_GROUPS, open_arm, close_until_collision,
    get_gripper_link, get_gripper_joints, get_disabled_collisions,
    COMPACT_LEFT_ARM, rightarm_from_leftarm, close_arm
)
from pybullet_tools.utils import (
    connect, disconnect, add_data_path, load_model, load_pybullet, set_pose, assign_link_colors,
    plan_joint_motion, set_joint_positions, get_pose, get_link_pose, multiply, Pose, stable_z, enable_gravity,
    get_joint_positions, quat_from_euler, Euler, PI, HideOutput, LockRenderer,joints_from_names,
    wait_if_gui, add_fixed_constraint, remove_constraint, joint_from_name, RGBA, interpolate, set_color,
    link_from_name, approximate_as_prism, interpolate_poses, create_box, get_max_limit, get_min_limit
)
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
from pybullet_tools.ikfast.pr2.ik import get_if_info


def run_simulation(loop=80):
    for _ in range(loop):
        p.stepSimulation()


class Franka:
    def __init__(self, pose):
        self.stable_config = [0, -PI/4, 0, -3*PI/4, 0, PI/2, PI/4]
        with LockRenderer(), HideOutput(True):
            self.robot = load_pybullet(FRANKA_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            assign_link_colors(self.robot, max_colors=2, s=0.7, v=1.0)
            self.tool_link = link_from_name(self.robot, 'tool_link')
            self.gripper_joints = [joint_from_name(self.robot, 'panda_finger_joint1'), joint_from_name(self.robot, 'panda_finger_joint2')]
            self.info = PANDA_INFO
            self.ik_joints = get_ik_joints(self.robot, self.info, self.tool_link)
            set_joint_positions(self.robot, self.ik_joints, self.stable_config)
            self.stable_pose = get_link_pose(self.robot,self.tool_link)
            self.constraint = None
        

    def move_to_pose(self, target_pose):
        tool_pose = get_link_pose(self.robot, self.tool_link)
        pose_path = interpolate_poses(tool_pose, target_pose, pos_step_size=0.012)
        for pose in pose_path:
            conf = next(either_inverse_kinematics(
                self.robot, self.info, self.tool_link, pose,
                use_pybullet=False, max_distance=1.2, max_time=0.5,
                max_candidates=100, verbose=False
            ), None)
            if conf is None:
                print('Unable to find IK solution for Franka.')
                return
            set_joint_positions(self.robot, self.ik_joints, conf)
            run_simulation()
    
    def open_gripper(self):
        open_conf = [get_max_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, open_conf)
        run_simulation()

    def close_gripper(self):
        close_conf = [get_min_limit(self.robot, joint) for joint in self.gripper_joints]
        set_joint_positions(self.robot, self.gripper_joints, close_conf)
        run_simulation()
    
    def grasp_gripper(self, obj):
        if self.constraint is None:
            close_until_collision(self.robot, self.gripper_joints, bodies=[obj])
            run_simulation(200)
            self.constraint = add_fixed_constraint(obj, self.robot, self.tool_link)
            run_simulation(200)
        else:
            print('Franka: Gripper not free')

    def release_gripper(self):
        if self.constraint is not None:
            run_simulation(200)
            remove_constraint(self.constraint)
            self.constraint = None
            self.open_gripper()
            run_simulation(160)
        else:
            print('Franka: Gripper is empty')

    def reset_arm(self, num_steps=50, close_grip=True):
        current_conf = get_joint_positions(self.robot, self.ik_joints)
        joint_path =  interpolate(current_conf, self.stable_config, num_steps=num_steps)
        for conf in joint_path:
            set_joint_positions(self.robot, self.ik_joints, conf)
            run_simulation()
        if close_grip is True:
            self.close_gripper()
    
    def get_grasp_pose(self, obj):
        body_pose = get_pose(obj)
        center, (w,l,height) =  approximate_as_prism(obj, body_pose=body_pose)
        pick_pose = multiply((center,body_pose[1]), Pose(point=[0, 0, 0.5 * height - 0.02]), Pose(euler=[0., PI, 0.]), Pose(euler=[0., 0., PI/2]))
        return pick_pose

    def get_lift_pose(self, grasp_pose):
        return (grasp_pose[0][0], grasp_pose[0][1], self.stable_pose[0][2]),grasp_pose[1]
    
    def get_place_pose(self, obj, place_mark, place_surface):
        place_mark = tuple(place_mark[:2]) + (stable_z(obj, place_surface),)
        body_pose = get_pose(obj)
        center, (w, length, height) = approximate_as_prism(obj, body_pose= (place_mark,body_pose[1]))
        pick_pose = multiply((center,body_pose[1]), Pose(point=[0, 0, 0.5 * height - 0.02]), Pose(euler=[0., PI, 0.]),Pose(euler=[0., 0., PI/2]))
        return pick_pose
    
    def pick_up(self, obj):
        self.open_gripper()
        franka_pick_pose = self.get_grasp_pose(obj)
        franka_lift_pose = self.get_lift_pose(franka_pick_pose)
        self.move_to_pose(franka_lift_pose)
        run_simulation(200)
        self.move_to_pose(franka_pick_pose)
        self.grasp_gripper(obj)
        self.move_to_pose(franka_lift_pose)

    def place(self, obj, location, surface):
        franka_place_pose = self.get_place_pose(obj, location, surface)
        franka_lift_pose = self.get_lift_pose(franka_place_pose)
        self.move_to_pose(franka_place_pose)
        self.release_gripper()
        self.move_to_pose(franka_lift_pose)

    def pick_and_place(self, obj, place_loc, place_surf):
        self.pick_up(obj)
        run_simulation(200)
        self.reset_arm(close_grip=False)
        run_simulation(200)
        self.place(obj, place_loc, place_surf)
        self.reset_arm()
        

class PR2:
    def __init__(self, pose, planning_arm='right'):
        with LockRenderer(), HideOutput(True):
            self.robot = load_model(DRAKE_PR2_URDF, fixed_base=True)
            set_pose(self.robot, pose)
            set_joint_positions(self.robot, joints_from_names(self.robot,PR2_GROUPS['left_arm']), COMPACT_LEFT_ARM)
            set_joint_positions(self.robot, joints_from_names(self.robot,PR2_GROUPS['right_arm']), rightarm_from_leftarm(COMPACT_LEFT_ARM))
            self.select_arm(planning_arm)
            self.constraint = None
            self.gripper_joints = get_gripper_joints(self.robot, self.arm)

    def select_arm(self, planning_arm):
        self.arm = planning_arm
        self.tool_link = get_gripper_link(self.robot, self.arm)
        self.arm_info = get_if_info(self.arm)
        self.ik_joints = get_ik_joints(self.robot, self.arm_info, self.tool_link)
        self.gripper_joints = get_gripper_joints(self.robot, self.arm)

    def plan_base_motion(self, goal_conf, obstacles=[]):
        grip_conf = get_joint_positions(self.robot, self.gripper_joints)
        arm_conf = get_joint_positions(self.robot, self.ik_joints)
        disabled = get_disabled_collisions(self.robot)
        base_joints = [joint_from_name(self.robot, name) for name in PR2_GROUPS['base']]
        with LockRenderer():
            base_path = plan_joint_motion(self.robot, base_joints[:2], goal_conf[:2],
                                      obstacles=obstacles, disabled_collisions=disabled)
        if base_path is None:
            print("PR2: base path not found")
            return
        for q in base_path:
            set_joint_positions(self.robot, base_joints[:2], q)
            set_joint_positions(self.robot, self.gripper_joints, grip_conf)
            set_joint_positions(self.robot, self.ik_joints, arm_conf)
            run_simulation()

    def arm_motion(self, target_pose):
        tool_pose = get_link_pose(self.robot, self.tool_link)
        pose_path = interpolate_poses(tool_pose, target_pose, pos_step_size=0.015)
        for pose in pose_path:
            conf = next(either_inverse_kinematics(
                self.robot, self.arm_info, self.tool_link, pose, fixed_joints=[self.ik_joints[0]],
                use_pybullet=False, max_distance=1.2, max_time=0.3,
                max_candidates=100, verbose=False
            ), None)
            if conf is None:
                print("PR2: IK not found")
                return
            set_joint_positions(self.robot, self.ik_joints, conf)
            run_simulation()

    def open_gripper(self):
        open_arm(self.robot, self.arm)
        run_simulation()
    
    def close_gripper(self):
        close_arm(self.robot, self.arm)
        run_simulation()

    def grasp_gripper(self, obj):
        if self.constraint is None:
            close_until_collision(self.robot, self.gripper_joints, bodies=[obj])
            run_simulation(200)
            self.constraint = add_fixed_constraint(obj, self.robot, get_gripper_link(self.robot, self.arm))
            run_simulation(200)
        else:
            print("PR2: Gripper not free")

    def release_gripper(self):
        if self.constraint is not None:
            run_simulation(200)
            remove_constraint(self.constraint)
            self.constraint = None
            self.open_gripper()
            run_simulation(200)
        else:
            print("PR2: Gripper is empty")


    def reset_arm(self, num_steps=25):
        goal_conf = COMPACT_LEFT_ARM if self.arm == 'left' else rightarm_from_leftarm(COMPACT_LEFT_ARM)
        current_conf = get_joint_positions(self.robot, self.ik_joints[1:])
        joint_path =  interpolate(current_conf, goal_conf, num_steps=num_steps)
        for conf in joint_path:
            set_joint_positions(self.robot, self.ik_joints[1:], conf)
            run_simulation()
        self.close_gripper()
        

    
    def get_grasp_pose(self, obj):
        body_pose = get_pose(obj)
        center, (w, length, height) = approximate_as_prism(obj, body_pose= body_pose)
        pick_pose = multiply((center,body_pose[1]), Pose(point=[0.045 - 0.5 * length, 0., 0.5 * height - 0.02]))
        return pick_pose
    
    def grasp_approach_base(self, place_mark):
        base_pose = get_pose(self.robot)
        offset =  -0.2 if self.arm == 'left' else 0.2
        return tuple(map(lambda x, y: x + y, (-0.85,offset,0.), tuple(place_mark[:2]) + (base_pose[0][2],)))
    
    def get_lift_pose(self, grasp_pose):
        return multiply(grasp_pose, Pose(point=[-0.02, 0.0, 0.14]))
    
    def get_place_pose(self, obj, place_mark, place_surface):
        place_mark = tuple(place_mark[:2]) + (stable_z(obj, place_surface),)
        body_pose = get_pose(obj)
        center, (w, length, height) = approximate_as_prism(obj, body_pose= (place_mark,body_pose[1]))
        pick_pose = multiply((center,body_pose[1]), Pose(point=[0.045 - 0.5 * length, 0., 0.5 * height - 0.02]))
        return pick_pose
    
    def pick_up(self, obj):
        grasp_pose = self.get_grasp_pose(obj)
        lift_pose = self.get_lift_pose(grasp_pose)
        self.open_gripper()
        self.arm_motion(lift_pose)
        run_simulation(200)
        self.arm_motion(grasp_pose)
        self.grasp_gripper(obj)
        self.arm_motion(lift_pose)

    def place(self, obj, location, surface):
        place_pose = self.get_place_pose(obj, location, surface)
        lift_pose = self.get_lift_pose(place_pose)
        self.arm_motion(place_pose)
        self.release_gripper()
        self.arm_motion(lift_pose)


    def pick_and_place(self, obj, place_loc, place_surf, obstacles=[]):
        obj_pose = get_pose(obj)
        goal1 = self.grasp_approach_base(obj_pose[0])
        goal2 = self.grasp_approach_base(place_loc)
        self.plan_base_motion(goal1, obstacles=obstacles)
        self.pick_up(obj)
        self.plan_base_motion(goal2, obstacles=obstacles)
        self.place(obj, place_loc, place_surf)
        self.reset_arm()


class Env:
    def __init__(self, use_gui=True):
        connect(use_gui=use_gui)
        add_data_path()
        self._setup_scene()
        self.franka = Franka(self.franka_pose)
        self.pr2 = PR2(self.pr2_pose, 'left')

    def _setup_scene(self):
        self.plane = p.loadURDF("plane.urdf")

        self.franka_pose = Pose(point=[1.8, 3.5, 0.625])
        self.pr2_pose = Pose()
        table1_pose = ([2.5, 1.2, 0.], quat_from_euler(Euler(yaw=PI / 2)))
        table2_pose = ([2., 3., 0.], quat_from_euler(Euler(yaw=PI / 2)))
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

        block1 = load_pybullet('models/drake/objects/block_for_pick_and_place_small.urdf', fixed_base=False)
        set_color(block1, RGBA(0.7,0.7,0.2,1.))
        set_pose(block1, Pose(point=[2.3,1.4,stable_z(block1, table1)]))
        self.block1 = block1

        block2 = load_pybullet('models/drake/objects/block_for_pick_and_place_small.urdf', fixed_base=False)
        set_color(block2, RGBA(0.6,0.6,0.6,1.))
        set_pose(block2, Pose(point=[2.1,1., stable_z(block2, table1)]))
        self.block2 = block2

        block3 = load_pybullet('models/drake/objects/block_for_pick_and_place_small.urdf', fixed_base=False)
        set_color(block3, RGBA(0.1,0.5,0.1,1.))
        set_pose(block3, Pose(point=[2.2,0.8,stable_z(block3, table1)]))
        self.block3 = block3

        #enable_gravity()

        self.cup, self.table1, self.table2, self.plate = cup, table1, table2, plate

    def execute_task(self):
        pr2, franka, table1, table2, plane = self.pr2, self.franka, self.table1, self.table2, self.plane
        block1, block2, block3, cup = self.block1, self.block2, self.block3, self.cup

        pr2.pick_and_place(block1, self.common_place_location, table2, obstacles=[table1, table2, franka.robot, plane, block2, block3, cup])

        franka.pick_and_place(block1, self.franka_place_location, table2)

        pr2.pick_and_place(block2, self.common_place_location, table2, obstacles=[table1, table2, franka.robot, plane, block3, cup])

        franka.pick_and_place(block2, self.franka_place_location, block1)

        pr2.pick_and_place(block3, self.common_place_location, table2, obstacles=[table1, table2, franka.robot, plane, cup])

        franka.pick_and_place(block3, self.franka_place_location, block2)

        print("Task execution complete.")



def main():
    env = Env(use_gui=True)
    
    run_simulation(200)

    wait_if_gui('Start?')
    
    env.execute_task()

    run_simulation(200)

    wait_if_gui('Finish?')
    disconnect()


if __name__ == '__main__':
    main()
