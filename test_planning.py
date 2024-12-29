import os
import sys
import numpy as np
import argparse
from termcolor import cprint
import pybullet as p

from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info

HERE = os.path.dirname(__file__)
robot_path = os.path.join(HERE, 'minicobo_v14', 'urdf', 'minicobo.urdf')
workspace_path = os.path.join(HERE, 'pybullet_planning_tutorials','examples','data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')

def jaka_demo():
    # * this will create the pybullet GUI
    # setting viewers=False will enter GUI-free mode
    connect(use_gui=True)
    cprint("Welcome to pybullet! <Ctrl+left mouse> to rotate, <Ctrl+middle mouse> to move the camera, <Ctrl+right mouse> to zoom", 'green')
    cprint('But the space is empty, let\'s load our robot!', 'yellow')
    # wait_for_user is your friend! It will pause the console, but having a separate thread keeping
    # the GUI running so you can rotate and see
    wait_for_user()

    robot = load_pybullet(robot_path, fixed_base=True)
    # workspace = load_pybullet(workspace_path, fixed_base=True)

    # this will print all the bodies' information in your console
    dump_world()
    cprint('You just loaded a robot, a workspace (with many static objects as its link, I modeled our good old MIT 3-412 shop here), '
            + 'and an end effector (it\'s inside the robot base now)', 'green')
    wait_for_user()

    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
        set_camera_pose(tuple(camera_pt), camera_base_pt)
    
    # * each joint of the robot are assigned with an integer in pybullet
    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)
    cprint('Joint {} \ncorresponds to:\n{}'.format(ik_joints, ik_joint_names), 'green')

    # * get a joint configuration sample function:
    # it separately sample each joint value within the feasible range
    sample_fn = get_sample_fn(robot, ik_joints)

    robot_start_conf = sample_fn()
    cprint(f"This is before updating pose: {robot_start_conf}", 'yellow')
    wait_for_user()
    # * set joint configuration, the robot's pose will be updated
    set_joint_positions(robot, ik_joints, robot_start_conf)
    cprint("This is after set joint pose: {}".format(robot_start_conf), 'green')
    wait_for_user()

    robot_end_conf = list(sample_fn())

    path = plan_joint_motion(robot, ik_joints, robot_end_conf, self_collisions=True)
    if path is None:
        cprint('no plan found', 'red')
    else:
        wait_for_user('a motion plan is found! Press enter to start simulating!')

    # adjusting this number will adjust the simulation speed
    time_step = 0.03
    for conf in path:
        set_joint_positions(robot, ik_joints, conf)
        wait_for_duration(time_step)
    
    wait_for_user('Simulation finished! Press enter to disconnect.')
    disconnect()

def move_to(robot, target_pos, target_orn=None):
    ik_joints = get_movable_joints(robot)
    robot_end_pos = target_pos
    robot_end_rot = [180, 0, -40] if target_orn is None else target_orn
    robot_end_quat = p.getQuaternionFromEuler(robot_end_rot)
    # use ik to get the end conf
    robot_end_conf = p.calculateInverseKinematics(robot, 7, robot_end_pos)#, targetOrientation=robot_end_quat)
    cprint(f"The joints of target pos: {robot_end_conf}", 'yellow')

    path = plan_joint_motion(robot, ik_joints, robot_end_conf, self_collisions=True)
    if path is None:
        cprint('no plan found', 'red')
    else:
        wait_for_user('a motion plan is found! Press enter to start simulating!')

    # adjusting this number will adjust the simulation speed
    time_step = 0.03
    for conf in path:
        set_joint_positions(robot, ik_joints, conf)
        wait_for_duration(time_step)

def jaka_to_block():
    # * this will create the pybullet GUI
    # setting viewers=False will enter GUI-free mode
    connect(use_gui=True, color=[0.8, 0.8, 0.8])
    cprint("Welcome to pybullet! <Ctrl+left mouse> to rotate, <Ctrl+middle mouse> to move the camera, <Ctrl+right mouse> to zoom", 'green')
    cprint('But the space is empty, let\'s load our robot!', 'yellow')
    # wait_for_user is your friend! It will pause the console, but having a separate thread keeping
    # the GUI running so you can rotate and see
    wait_for_user()

    robot = load_pybullet(robot_path, fixed_base=True)
    # workspace = load_pybullet(workspace_path, fixed_base=True)

    # this will print all the bodies' information in your console
    dump_world()


    cprint('You just loaded a robot, a workspace (with many static objects as its link, I modeled our good old MIT 3-412 shop here), '
            + 'and an end effector (it\'s inside the robot base now)', 'green')
    wait_for_user()

    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
        set_camera_pose(tuple(camera_pt), camera_base_pt)
    
    # * each joint of the robot are assigned with an integer in pybullet
    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)
    cprint('Joint {} \ncorresponds to:\n{}'.format(ik_joints, ik_joint_names), 'green')

    # * get a joint configuration sample function:
    # it separately sample each joint value within the feasible range
    sample_fn = get_sample_fn(robot, ik_joints)

    PI = 3.14
    robot_start_conf = [PI,PI/2,0,PI//4,0,0]
    # robot_start_pos = [-0.08, -0.220, 0.127]
    # robot_start_rot = [180, 0, -40]
    # robot_start_quat = p.getQuaternionFromEuler(robot_start_rot)
    # print("quat: ", robot_start_quat)
    # # use ik to get the end conf
    # robot_start_conf = p.calculateInverseKinematics(robot, 7, robot_start_pos)#, targetOrientation=robot_end_quat)
    print("Start joints: ",robot_start_conf)
    
    cprint(f"This is before updating pose: {robot_start_conf}", 'yellow')
    wait_for_user()
    # * set joint configuration, the robot's pose will be updated
    set_joint_positions(robot, ik_joints, robot_start_conf)
    tcp_position = get_link_pose(robot, 7)
    cprint("This is after set joint pose: {}".format(robot_start_conf), 'green')
    cprint(f"TCP position: {tcp_position}", 'green')
    wait_for_user()

    robot_end_pos = [-0.08, -0.360, 0.127]
    robot_end_rot = [180, 0, -40]
    move_to(robot, robot_end_pos)
    wait_for_user()

    robot_end_pos = [0.360, -0.08, 0.127]
    robot_end_rot = [180, 0, -40]
    move_to(robot, robot_end_pos)
    
    tcp_position = get_link_pose(robot, 7)
    cprint(f"TCP position: {tcp_position}", 'green')
    tcp_position = get_link_pose(robot, 6)
    cprint(f"link6 position: {tcp_position}", 'green')

    wait_for_user('Simulation finished! Press enter to disconnect.')
    disconnect()

def joints_to_tcp_pose(robot, joints):
    current_conf = get_joint_positions(robot, get_movable_joints(robot))

    ik_joints = get_movable_joints(robot)
    assert len(joints) == len(ik_joints)
    robot_end_conf = joints
    set_joint_positions(robot, ik_joints, robot_end_conf)
    tcp_pose = get_link_pose(robot, 7)
    set_joint_positions(robot, ik_joints, current_conf)
    return tcp_pose


jaka_to_block() 