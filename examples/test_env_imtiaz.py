#!/usr/bin/env python

from __future__ import print_function

import pybullet as p

from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, get_torso_arm_joints, get_gripper_link, get_arm_joints,\
    PR2_GROUPS, open_arm, close_until_collision, get_gripper_joints, get_disabled_collisions, COMPACT_LEFT_ARM, rightarm_from_leftarm
from pybullet_tools.utils import joint_from_name, quat_from_euler, draw_pose, sub_inverse_kinematics,pairwise_collision,set_joint_positions,\
    add_data_path, connect, plan_joint_motion, approximate_as_cylinder,interpolate_poses, load_model, joints_from_names, wait_if_gui,BLOCK_URDF,\
    disconnect, get_joint_positions, link_from_name, get_link_pose, enable_gravity, HideOutput, get_pose, wait_if_gui, load_pybullet,stable_z,\
    Euler, PI, wait_for_duration, LockRenderer, base_aligned_z, Point,Pose, assign_link_colors, multiply, set_pose, add_fixed_constraint, remove_constraint
    
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver
from pybullet_tools.ikfast.pr2.ik import sample_tool_ik, is_ik_compiled, pr2_inverse_kinematics, get_if_info
from pybullet_tools.ikfast.utils import USE_CURRENT

def test_base_motion(pr2, base_start, base_goal, obstacles=[]):
    disabled_collisions = get_disabled_collisions(pr2)
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    set_joint_positions(pr2, base_joints, base_start)
    base_joints = base_joints[:2]
    base_goal = base_goal[:len(base_joints)]
    wait_if_gui('Plan Base?')
    with LockRenderer(lock=True):
        base_path = plan_joint_motion(pr2, base_joints, base_goal, obstacles=obstacles, disabled_collisions=disabled_collisions)
    if base_path is None:
        print('Unable to find a base path')
        return
    print('Number of waypoints: '+str(len(base_path)))
    for bq in base_path:
        set_joint_positions(pr2, base_joints, bq)
        for _ in range(100):
            p.stepSimulation()
        

def pr2_ik(robot, arm, gripper_pose, obstacles=[], custom_limits={}, use_pybullet=False,**kwargs):
    arm_link = get_gripper_link(robot, arm)
    arm_joints = get_arm_joints(robot, arm)
    ik_joints = get_torso_arm_joints(robot, arm)
    current_torso_conf = get_joint_positions(robot, ik_joints)
    if not use_pybullet and is_ik_compiled():
        
        torso_arm_conf = sample_tool_ik(robot, arm, gripper_pose, custom_limits=custom_limits,
                                        torso_limits=USE_CURRENT, **kwargs)
        if torso_arm_conf is None:
            return None
        set_joint_positions(robot, ik_joints, torso_arm_conf)
    else:
        arm_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, gripper_pose, custom_limits=custom_limits)
        if arm_conf is None:
            return None
    if any(pairwise_collision(robot, b) for b in obstacles):
        return None
    temp = get_joint_positions(robot, ik_joints)
    set_joint_positions(robot, ik_joints, current_torso_conf)
    wait_for_duration(0.01)
    return temp 
    
def test_torso_arm_motion(pr2, arm, grasp_pose, obstacles=[]):
    torso_arm_joints = get_torso_arm_joints(pr2, arm)
    disabled_collisions = get_disabled_collisions(pr2)
    wait_if_gui('Plan arm with ik?')
    draw_pose(get_link_pose(pr2,get_gripper_link(pr2,arm)))
    draw_pose(grasp_pose)
    flag = False
    ik_sol = None
    max_iter = 500
    count = 0
    while flag is False:
        count+=1
        ik_sol = pr2_ik(pr2, arm, grasp_pose, obstacles=obstacles)
        if ik_sol is not None:
            with LockRenderer(lock=False):
                arm_path = plan_joint_motion(pr2, torso_arm_joints, ik_sol, obstacles=obstacles, disabled_collisions=disabled_collisions)
            if arm_path is not None:
                flag =  True
        if count>max_iter and flag is False:
            print('unable to find a solution')
            return
    set_joint_positions(pr2, torso_arm_joints, arm_path[0])
    wait_if_gui('Move Arm?')
    print('Number of waypoints: '+str(len(arm_path)))
    for q in arm_path:
        set_joint_positions(pr2, torso_arm_joints, q)
        wait_for_duration(0.05)
    
          
def test_franka_motion(robot, info, tool_link, target_pose):
    tool_pose = get_link_pose(robot, tool_link)
    
    wait_if_gui('Move Franka Arm?')
    #draw_pose(tool_pose)
    #draw_pose(target_pose)
    
    check_ik_solver(info)
    ik_joints = get_ik_joints(robot, info, tool_link)
    
    pose_path = interpolate_poses(tool_pose, target_pose, pos_step_size=0.025)
    for pose in pose_path:
        conf = next(either_inverse_kinematics(robot, info, tool_link, pose, use_pybullet=False, max_distance=1.5, max_time=0.5, 
                      max_candidates=100, verbose=False),None)
        if conf is None:
            print('unable to find ik solution')
            return
        set_joint_positions(robot, ik_joints, conf)
        for _ in range(200):
            p.stepSimulation()
    print('Completed Franka Motion.')

def pr2_arm_motion(pr2, arm, target_pose):
    wait_if_gui('Move Arm?')
    tool_link = get_gripper_link(pr2, arm)
    tool_pose = get_link_pose(pr2, tool_link)
    info = get_if_info(arm)
    pose_path =  interpolate_poses(tool_pose, target_pose, pos_step_size = 0.025)
    ik_joints = get_ik_joints(pr2, info ,tool_link)
    for pose in pose_path:
        conf = next(either_inverse_kinematics(pr2, info, tool_link, pose, use_pybullet=False, max_distance=1., max_time=0.2, 
                      max_candidates=100, verbose=False),None)
        if conf is None:
            print('unable to find ik solution')
            return
        set_joint_positions(pr2, ik_joints, conf)
        for _ in range(100):
            p.stepSimulation()      
    
def main():
    
    connect(use_gui=True)
    add_data_path()
    
    table_pose = ([2.,3.,0.],quat_from_euler(Euler(yaw=PI/2)))
    plate_pose = Pose(point=[2.2,3.5,0.625])
    cup_pose = Pose(point=[1.8,2.5, 0.625])
    franka_pose = Pose(point=([1.8,3.5,0.625]))
    planning_arm = 'right'
    
    
    b_start = (0.,0.,0.)
    b_goal = (0.9,2.7,0.)
    b_goal2 = (0.8,3.4,0.)
    
    
    plane = p.loadURDF("plane.urdf")
    table_path = "models/table_collision/table.urdf"
    table = load_pybullet(table_path, fixed_base=True)
    set_pose(table, table_pose) 
    plate_path ="models/dinnerware/plate.urdf"
    plate = load_pybullet(plate_path, fixed_base=True)
    set_pose(plate, plate_pose)
    cup_path = "models/dinnerware/cup/cup_small.urdf"
    cup = load_pybullet(cup_path, fixed_base=False)
    cup_pose = Pose(point=[1.8,2.5,stable_z(cup, table)])
    set_pose(cup, cup_pose)
    
    with LockRenderer():
        with HideOutput(True):
            arm = load_pybullet(FRANKA_URDF, fixed_base=True)
            set_pose(arm, franka_pose)
            assign_link_colors(arm, max_colors=3, s=0.5, v=1.)
            arm_link = link_from_name(arm, 'tool_link')
            pr2 = load_model(DRAKE_PR2_URDF, fixed_base=True)
            set_pose(pr2, Pose(point=Point(z=base_aligned_z(pr2))))
            set_joint_positions(pr2, joints_from_names(pr2, PR2_GROUPS['left_arm']), COMPACT_LEFT_ARM)
            set_joint_positions(pr2, joints_from_names(pr2, PR2_GROUPS['right_arm']), rightarm_from_leftarm(COMPACT_LEFT_ARM))
    
    test_base_motion(pr2, b_start, b_goal, obstacles=[table, plate, cup, arm, plane])

    wait_if_gui('Open Gripper?')
    open_arm(pr2, planning_arm)
    
    center,(diameter, height) = approximate_as_cylinder(cup)
    grasp_pose= multiply(get_pose(cup),Pose(point=[-0.1*diameter,0.,0.5*height]))
    
    #test_torso_arm_motion(pr2, planning_arm ,grasp_pose, obstacles=[table, cup, plane])
    pr2_arm_motion(pr2, planning_arm, grasp_pose)   
    wait_if_gui('Close Gripper?')
    
    close_until_collision(pr2,get_gripper_joints(pr2,planning_arm),bodies=[cup])
    
    c1 = add_fixed_constraint(cup,pr2,get_gripper_link(pr2,planning_arm))
    for _ in range(5000):
        p.stepSimulation()
    wait_if_gui('Pick Up?')
    #test_torso_arm_motion(pr2, planning_arm,multiply(grasp_pose,Pose(euler=[0.,-PI/9,0.]),Pose(point=[0,0,height*2]),Pose(euler=[0.,PI/9,0.])),
#                          obstacles=[table,plane])
    pr2_arm_motion(pr2, planning_arm,multiply(grasp_pose,Pose(point=[-0.1*diameter,0,height*2.])))
    
    test_base_motion(pr2, b_goal, b_goal2, obstacles=[table, plate, cup, arm, plane])
    
    franka_curr_pose =  get_link_pose(arm,arm_link)
    franka_pick_pose =  multiply(get_pose(cup),Pose(point=[0,0,1.*height]),Pose(euler=[0.,PI,0.]),Pose(euler=[0.,0.,PI/2]))
    franka_pre_pose = multiply(franka_pick_pose,Pose(point=[0,0,-6.*height]))
    franka_place_pose = multiply(get_pose(plate),Pose(point=[0,0,2.*height]),Pose(euler=[0.,PI,0.]))
    
    
    test_franka_motion(arm,PANDA_INFO,arm_link,franka_pick_pose)
    
    draw_pose(franka_pick_pose)
    draw_pose(franka_pre_pose)
    draw_pose(get_link_pose(arm,arm_link))
    
    wait_if_gui('Attach?')
    c2 =  add_fixed_constraint(cup, arm, arm_link)
    remove_constraint(c1)
    open_arm(pr2,planning_arm)
    for _ in range(1000):
        p.stepSimulation()
    
    test_franka_motion(arm,PANDA_INFO,arm_link,franka_pre_pose)
    test_franka_motion(arm,PANDA_INFO,arm_link,franka_place_pose)
    
    remove_constraint(c2)
    enable_gravity()
    for _ in range(1000):
        p.stepSimulation()
    
    test_franka_motion(arm,PANDA_INFO,arm_link,franka_curr_pose)
    
    wait_if_gui('Finish?')
    
    disconnect()

if __name__ == '__main__':
    main()
