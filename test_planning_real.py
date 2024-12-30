# Writen by Wang Ziyu @ 2024-12-29
import cv2
from pybullet_planning import compute_inverse_kinematics, get_ik_tool_link_pose
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
import os
import sys
import numpy as np
import argparse
from termcolor import cprint
import pybullet as p
import requests as req
sys.path.append(os.path.join(os.path.dirname(__file__), './SDK'))


HERE = os.path.dirname(__file__)
robot_path = os.path.join(HERE, 'minicobo_v14', 'urdf', 'minicobo.urdf')
workspace_path = os.path.join(HERE, 'pybullet_planning_tutorials', 'examples',
                              'data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')
FIX_QUAT = [-0.4717700481414795, 0.8794430494308472, 0, 0]


# 运动模式
ABS = 0
INCR = 1


class RobotController:
    def __init__(self):
        self.robot = None
        self.matrix = None
        self.jaka = None
        self.cap = cv2.VideoCapture(2)
        connect(use_gui=True, color=[0.8, 0.8, 0.8])

        # ==========标定相机与机器人坐标系之间的关系==========
        # 像素坐标系下的四个角点坐标
        self.pixel_points = np.array([
            [105, 165],
            [200, 100],
            [200, 200],
            [100, 200]
        ], dtype=np.float32)

        # 机器人坐标系下的四个角点坐标
        self.robot_points = np.array([
            [0, 0],
            [1, 0],
            [1, 1],
            [0, 1]
        ], dtype=np.float32)

        # 计算透视变换矩阵
        self.matrix = cv2.getPerspectiveTransform(
            self.pixel_points, self.robot_points)
        # ==========标定相机与机器人坐标系之间的关系==========

        self.FINISH_POINT = []  # TODO
        self.READY_POINT = []

    def load_jaka(self, robot_path):
        self.robot = load_pybullet(robot_path, fixed_base=True)
        self.ik_joints = get_movable_joints(self.robot)

    def connect_robot(self):
        import jkrc
        self.jaka = jkrc.RC("10.5.5.100")
        ret = self.jaka.login()
        print("login: ", ret)
        power = self.jaka.power_on()
        print("power: ", power)
        self.jaka.enable_robot()
        self.jaka.drag_mode_enable(True)
        ret = self.jaka.is_in_drag_mode()
        print("drag mode: ", ret)

    def updatae_matrix(self):
        self.matrix = cv2.getPerspectiveTransform(
            self.pixel_points, self.robot_points)

    def pick_four_points(self):
        # 拍摄一张图片， 框选四个角点
        id = 1
        img = self.get_rgb_image(id)
        while img is None:
            print("Failed to get image at camera ", id)
            id += 1
            img = self.get_rgb_image(id)
        cv2.imshow('frame', img)

        robot_points = []
        # Mouse callback function

        # cap.release()
        

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                # Add the clicked point to the robot_points array
                robot_points.append([x, y])
                # Display the clicked point on the image
                cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow('frame', img)

        # Register the mouse callback function
        cv2.setMouseCallback('frame', mouse_callback)

        # Wait for the user to click four points
        while len(robot_points) < 4:
            # wait for the user to click the four points
            cv2.waitKey(1)
        cv2.destroyAllWindows()

        return robot_points

    def calibrate_matrix(self):
        # calibrate the FINISH_POINT and READY_POINT
        wait_for_user('Please move to the FINISH_POINT.')
        joints = self.jaka.get_joint_position()[1]
        print("current joints: ", joints)
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.FINISH_POINT = tcp_position

        wait_for_user('Please move to the READY_POINT.')
        joints = self.jaka.get_joint_position()[1]
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.READY_POINT = tcp_position

        # Calibrate the rgb position to robot position
        wait_for_user('Begin matrix calib.')
        self.robot_points = np.array(self.pick_four_points(), dtype=np.float32)

        # Print the four corner points
        print("Four corners are at", self.robot_points)

        wait_for_user(
            'Please move the robot to the first corner point and press enter.')
        joints = self.jaka.get_joint_position()[1]
        print("current joints: ", joints)
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.robot_points[0] = tcp_position[:2]

        wait_for_user(
            'Please move the robot to the second corner point and press enter.')
        joints = self.jaka.get_joint_position()[1]
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.robot_points[1] = tcp_position[:2]

        wait_for_user(
            'Please move the robot to the third corner point and press enter.')
        joints = self.jaka.get_joint_position()[1]
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.robot_points[2] = tcp_position[:2]

        wait_for_user(
            'Please move the robot to the fourth corner point and press enter.')
        joints = self.jaka.get_joint_position()[1]
        tcp_position = self.joints_to_tcp_pose(joints)[0]
        self.robot_points[3] = tcp_position[:2]

        print("Four corners are at ", self.robot_points)

        self.updatae_matrix()

        self.jaka.drag_mode_enable(False)

        wait_for_user('Calibration finished! Press enter to continue.')

    def get_rgb_image(self, id=0):
        self.cap = cv2.VideoCapture(id)
        try:
            cap = self.cap
            ret, frame = cap.read()
            cv2.imshow('frame', frame)
            cap.release()
            cv2.destroyAllWindows()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame
        except Exception as e:
            print(f"Error: {e}")
            return None

    def get_image(self, id=0):
        self.cap = cv2.VideoCapture(id)
        try:
            cap = self.cap
            ret, frame = cap.read()
            cv2.imshow('frame', frame)
            cap.release()
            cv2.destroyAllWindows()
            return frame
        except Exception as e:
            print(f"Error: {e}")
            return None


    def move_to(self, target_pos, target_orn=None):
        robot_end_pos = target_pos
        robot_end_quat = FIX_QUAT if target_orn is None else target_orn
        robot_end_conf = p.calculateInverseKinematics(
            self.robot, 7, robot_end_pos, targetOrientation=robot_end_quat)
        path = plan_joint_motion(
            self.robot, self.ik_joints, robot_end_conf, self_collisions=True)
        if path is None:
            cprint('no plan found', 'red')
        else:
            wait_for_user(
                'a motion plan is found! Press enter to start simulating!')
        time_step = 0.03
        for conf in path:
            set_joint_positions(self.robot, self.ik_joints, conf)
            wait_for_duration(time_step)

    def jaka_to_block(self):
        self.load_jaka(robot_path)
        dump_world()
        wait_for_user()
        if has_gui():
            camera_base_pt = (0, 0, 0)
            camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
            set_camera_pose(tuple(camera_pt), camera_base_pt)
        ik_joint_names = get_joint_names(self.robot, self.ik_joints)
        cprint('Joint {} \ncorresponds to:\n{}'.format(
            self.ik_joints, ik_joint_names), 'green')
        sample_fn = get_sample_fn(self.robot, self.ik_joints)
        robot_start_pos = [-0.08, -0.360, 0.128]
        self.acc_move_to(robot_start_pos)
        wait_for_user()

        # set_joint_positions(self.robot, self.ik_joints, robot_start_conf)
        tcp_position = get_link_pose(self.robot, 7)
        cprint(f"TCP position: {tcp_position}", 'green')
        wait_for_user()

        robot_end_pos = [0.052, -0.380, 0.127]
        self.acc_move_to(robot_end_pos)
        tcp_position = get_link_pose(self.robot, 7)
        cprint(f"TCP position: {tcp_position}", 'green')
        wait_for_user()

        robot_end_pos = [0.051, -0.220, 0.127]
        self.acc_move_to(robot_end_pos)
        tcp_position = get_link_pose(self.robot, 7)
        cprint(f"TCP position: {tcp_position}", 'green')
        wait_for_user()

        robot_end_pos = [-0.07, -0.22, 0.127]
        self.acc_move_to(robot_end_pos)
        tcp_position = get_link_pose(self.robot, 7)
        cprint(f"TCP position: {tcp_position}", 'green')
        wait_for_user('Simulation finished! Press enter to disconnect.')

        disconnect()

    def joints_to_tcp_pose(self, joints):
        current_states = p.getJointStates(
            self.robot, get_movable_joints(self.robot))
        current_conf = []
        for i in range(len(current_states)):
            current_conf.append(current_states[i][0])
        print("Current conf: ", current_conf)
        assert len(joints) == len(self.ik_joints)
        robot_end_conf = joints
        set_joint_positions(self.robot, self.ik_joints, robot_end_conf)
        tcp_pose = get_link_pose(self.robot, 7)
        # set_joint_positions(self.robot, self.ik_joints, current_conf)
        return tcp_pose

    def pixel2robot(self, pixel_point):
        pixel_point = np.array(
            [pixel_point], dtype=np.float32).reshape(-1, 1, 2)
        robot_point = cv2.perspectiveTransform(pixel_point, self.matrix)
        print("Robot point: ", robot_point)
        return robot_point[0, 0]

    def get_pixel_point(self, img):
        url = "https://29a8dea0b5764c0ea29afc2b174a0c1f.apig.cn-north-4.huaweicloudapis.com/v1/infers/be923697-802b-4a58-82ba-d65be4dc1162"
        with open("./token.txt") as fp:
            token = fp.read()
        _, img_encoded = cv2.imencode('.jpg', img)
        resp = req.post(url, files={"images": img_encoded}, headers={"X-Auth-Token": token})
        boxes = resp.json().get("detection_boxes")
        if not boxes:
            raise ValueError("Detection failed")
        return [[(b+d)/2, (a+c)/2] for a,b,c,d in boxes]

    def accurateCalculateInverseKinematics(self, kukaId, endEffectorId, targetPos, targetRot=None, threshold=0.01, maxIter=100):
        # save the current state
        current_states = p.getJointStates(
            self.robot, get_movable_joints(self.robot))
        current_conf = []
        for i in range(len(current_states)):
            current_conf.append(current_states[i][0])

        kukaEndEffectorIndex = endEffectorId
        closeEnough = False
        iter = 0
        dist2 = 1e30
        while (not closeEnough and iter < maxIter):
            if targetRot is not None:
                jointPoses = p.calculateInverseKinematics(
                    kukaId, kukaEndEffectorIndex, targetPos, targetRot)
            else:
                jointPoses = p.calculateInverseKinematics(
                    kukaId, kukaEndEffectorIndex, targetPos)
            set_joint_positions(self.robot, self.ik_joints, jointPoses)
            ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
            newPos = ls[4]
            diff = [targetPos[0] - newPos[0], targetPos[1] -
                    newPos[1], targetPos[2] - newPos[2]]
            dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
            closeEnough = (dist2 < threshold)
            iter = iter + 1
            print("Num iter: "+str(iter) + "threshold: "+str(dist2))

        # restore the state
        set_joint_positions(self.robot, self.ik_joints, current_conf)
        return jointPoses

    def acc_move_to(self, target_pos, target_orn=None):
        robot_end_pos = target_pos
        robot_end_quat = FIX_QUAT if target_orn is None else target_orn
        robot_end_conf = self.accurateCalculateInverseKinematics(
            self.robot, 7, robot_end_pos, targetRot=robot_end_quat)
        path = plan_joint_motion(
            self.robot, self.ik_joints, robot_end_conf, self_collisions=True)
        if path is None:
            cprint('no plan found', 'red')
        else:
            wait_for_user(
                'a motion plan is found! Press enter to start simulating!')
        time_step = 0.03
        for conf in path:
            # print(conf)
            set_joint_positions(self.robot, self.ik_joints, conf)
            wait_for_duration(time_step)

    def send_joints(self, joints):
        self.jaka.joint_move(joints, ABS, True, 0.1)

    def real_move_to(self, target_pos, target_orn=None):
        robot_end_pos = target_pos
        robot_end_quat = FIX_QUAT if target_orn is None else target_orn
        robot_end_conf = self.accurateCalculateInverseKinematics(
            self.robot, 7, robot_end_pos, targetRot=robot_end_quat)
        path = plan_joint_motion(
            self.robot, self.ik_joints, robot_end_conf, self_collisions=True)
        if path is None:
            cprint('no plan found', 'red')
        else:
            wait_for_user(
                'a motion plan is found! Press enter to start simulating!')
        time_step = 0.1
        for conf in path:
            print(conf)
            set_joint_positions(self.robot, self.ik_joints, conf)
            wait_for_duration(time_step)
            self.send_joints(conf)
        # Add a delay after the motion to stabilize the robot
        # wait_for_duration(1.0)


def main():
    controller = RobotController()
    # controller.get_rgb_image()
    # controller.jaka_to_block()

    controller.load_jaka(robot_path)
    controller.connect_robot()
    wait_for_user('Connect robot successfully! Press enter to continue.')
    controller.calibrate_matrix()

    wait_for_user('Enter to photo the block.')
    id = 1
    img = controller.get_rgb_image(id)
    while img is None:
        print("Failed to get image at camera ", id)
        id += 1
        img = controller.get_rgb_image(id)
    # cv2.imshow('frame', img)

    pixel_points = controller.get_pixel_point(img)

    id = 1
    img = controller.get_image(id)
    while img is None:
        print("Failed to get image at camera ", id)
        id += 1
        img = controller.get_image(id)
    # cv2.imshow('frame', img)
    print("pixel_points:", pixel_points)
    for i in range(len(pixel_points)):
        x = int(pixel_points[i][0])
        y = int(pixel_points[i][1])
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow('frame', img)

        robot_point = controller.pixel2robot(pixel_points[i])
        print(f"Ready to move to pixel point: {robot_point}, idx", i)
        # TODO: enable the tool++++++++++++++++++

        wait_for_user('Enter to move robot to the block.')
        robot_point = [robot_point[0], robot_point[1], 0.127]
        controller.real_move_to(robot_point)
        wait_for_duration(1)
        # TODO: down the tool and grab the block, and add try catch

        print("Ready to move to finish point")
        controller.real_move_to(controller.FINISH_POINT)
        wait_for_duration(1)

        # TODO: disable the tool

        print("Ready to move back to ready point")
        controller.real_move_to(controller.READY_POINT)
        wait_for_duration(1)


def debug():
    controller = RobotController()
    # controller.get_rgb_image()
    # controller.jaka_to_block()

    controller.load_jaka(robot_path)
    controller.connect_robot()
    wait_for_user('Connect robot successfully! Press enter to continue.')
    controller.calibrate_matrix()

    wait_for_user('Enter to photo the block.')
    pixel_points = controller.pick_four_points()
    for i in range(len(pixel_points)):
        robot_point = controller.pixel2robot(pixel_points[i])
        print("Ready to move to pixel point: ", i)
        # TODO: enable the tool++++++++++++++++++

        wait_for_user('Enter to move robot to the block.')
        robot_point = [robot_point[0], robot_point[1], 0.127]
        controller.acc_move_to(robot_point)
        wait_for_duration(1)
        # TODO: down the tool and grab the block, and add try catch

        print("Ready to move to finish point")
        controller.acc_move_to(controller.FINISH_POINT)
        wait_for_duration(1)

        # TODO: disable the tool

        print("Ready to move back to ready point")
        controller.acc_move_to(controller.READY_POINT)
        wait_for_duration(1)


def debug_move():
    controller = RobotController()
    # controller.get_rgb_image()
    # controller.jaka_to_block()

    controller.load_jaka(robot_path)
    controller.connect_robot()
    wait_for_user('Connect robot successfully! Press enter to continue.')
    controller.calibrate_matrix()

    wait_for_user('Enter to move robot to the finish point')
    controller.real_move_to(controller.FINISH_POINT)
    wait_for_duration(1)

    # TODO: disable the tool

    wait_for_user('Enter to move robot to the ready point')
    controller.real_move_to(controller.READY_POINT)
    wait_for_duration(1)


if __name__ == '__main__':
    main()
    # debug_move()
