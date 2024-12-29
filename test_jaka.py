# test jaka

# export LD_LIBRARY_PATH=/home/osboxes/Desktop/GeneralRobot/SDK
# run before running this script

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), './SDK'))

import jkrc    
import time    
#坐标系    
COORD_BASE  = 0    
COORD_JOINT = 1    
COORD_TOOL  = 2    
#运动模式    
ABS = 0    
INCR= 1    
robot = jkrc.RC("10.5.5.100")    
# robot.set_debug_mode(1)
# print("Begin Debug!")

ret = robot.login()    
print("login: ", ret)


power = robot.power_on()  
print("power: ",power)  
robot.enable_robot()    
robot.drag_mode_enable(True)    
ret = robot.is_in_drag_mode()    
print(ret)    
while True:    
    a = input("input: ") 
    # when press 'a' key, get status of robot
    if a == 'a':
        # robot.drag_mode_enable(False)    
        joints = robot.get_joint_position()
        tcp_pos = robot.get_tcp_position()
        extend_io = robot.is_extio_running()
        # robot.drag_mode_enable(True)    
        tool_id = robot.get_tool_id()
        tool_ret = robot.set_digital_input(1, 0, 1)
        robot_stat = robot.get_robot_status ()
        # print("++++++++++++++++++++")
        # print("robot status: ", robot_stat)
        # print("++++++++++++++++++++")


        print("joints: ", joints)
        print("tcp_pos: ", tcp_pos)
        print("extend_io: ", extend_io)
        print("tool ret: ", tool_ret)
    elif a == 'm':
        PI = 3.14
        joint_pos=[PI,PI/2,0,PI//4,0,0]
        robot.joint_move(joint_pos,ABS,True,1)  

    elif a == 'q':
        break

# robot.drag_mode_enable(False)    
ret = robot.is_in_drag_mode()    
print(ret)    
robot.logout()
