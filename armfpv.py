import time
import numpy as np
from kinematics import transform
from common.ros_robot_controller_sdk import Board
from kinematics.kinematics_controller import KinematicsController

board = Board()
controller = KinematicsController(board)

###########inverse_kinematics##################
# [x, y, z(m)], pitch, [pitch_min, pitch_max](deg)获取运动学逆解([x, y, z(m)], pitch, [pitch_min, pitch_max](deg) get inverse kinematics solution)
# res = controller.get_all_ik([0, 0.1, 0.1], 0, [-180, 180])
print('ik input: \nxyz(m): {}\npitch(deg): {}\npitch_range(deg): {}\n'.format([0.2, 0, 0.25], 0, [-180, 180]))
res = controller.set_pose_target([0.2, 0, 0.25], 0, [-180, 180], roll=90)
print('ik output: \nik pulse: {}\ncurrent pulse: {}\nrpy{}\n'.format(*res[1:-1]))
if res[1]:
    board.bus_servo_set_position(1, [[2, res[1][4]], [3, res[1][3]], [4, res[1][2]], [5, res[1][1]], [6, res[1][0]]])
time.sleep(1)
res = controller.set_joint_value_target(*res[1])
print('fk output:\nxyz(m): {}\nqua: {}'.format(*res[2]))
print('rpy:', transform.qua2rpy(res[-1][-1]))
while True:
    for i in np.arange(0.25, 0.14, -0.01):
        controller.go_pose_target([0.18, 0, i], 0, [-180, 180], roll=np.interp(i, [0.15, 0.25], [-90, 90]), use_time=0.1)
        time.sleep(0.1)
    for i in np.arange(0.15, 0.26, 0.01):
        controller.go_pose_target([0.18, 0, i], 0, [-180, 180], roll=np.interp(i, [0.15, 0.25], [-90, 90]), use_time=0.1)
        time.sleep(0.1)
