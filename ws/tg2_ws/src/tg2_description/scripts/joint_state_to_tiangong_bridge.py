#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from bodyctrl_msgs.msg import CmdSetMotorPosition  # array wrapper
from bodyctrl_msgs.msg import SetMotorPosition     # single command

import math
from collections import defaultdict

# 关节名 -> (motor_id, group)
# group 用于区分要发到哪个话题：head/waist/arm/leg
JOINT_MAP = {
    # 头部（1..3）
    "head_roll":  (1,  "head"),
    "head_pitch": (2,  "head"),
    "head_yaw":   (3,  "head"),
    # 腰（31）
    "waist_yaw":  (31, "waist"),
    # 左臂（11..17）
    "l_shoulder_pitch": (11, "arm"),
    "l_shoulder_roll":  (12, "arm"),
    "l_shoulder_yaw":   (13, "arm"),
    "l_elbow_pitch":    (14, "arm"),
    "l_wrist_yaw":      (15, "arm"),
    "l_wrist_pitch":    (16, "arm"),
    "l_wrist_roll":     (17, "arm"),
    # 右臂（21..27）
    "r_shoulder_pitch": (21, "arm"),
    "r_shoulder_roll":  (22, "arm"),
    "r_shoulder_yaw":   (23, "arm"),
    "r_elbow_pitch":    (24, "arm"),
    "r_wrist_yaw":      (25, "arm"),
    "r_wrist_pitch":    (26, "arm"),
    "r_wrist_roll":     (27, "arm"),
    # 左腿（51..56）
    "l_hip_roll":   (51, "leg"),
    "l_hip_pitch":  (52, "leg"),
    "l_hip_yaw":    (53, "leg"),
    "l_knee_pitch": (54, "leg"),
    "l_ankle_pitch":(55, "leg"),
    "l_ankle_roll": (56, "leg"),
    # 右腿（61..66）
    "r_hip_roll":   (61, "leg"),
    "r_hip_pitch":  (62, "leg"),
    "r_hip_yaw":    (63, "leg"),
    "r_knee_pitch": (64, "leg"),
    "r_ankle_pitch":(65, "leg"),
    "r_ankle_roll": (66, "leg"),
}

# 角度限位（度）。来自文档表格；若缺失则不裁剪。
DEG_LIMITS = {
    # 头
    "head_roll":  (-26, 26),
    "head_pitch": (-25, 25),
    "head_yaw":   (-90, 90),
    # 腰
    "waist_yaw":  (-170, 170),
    # 左臂
    "l_shoulder_pitch": (-170, 170),
    "l_shoulder_roll":  (-15, 150),
    "l_shoulder_yaw":   (-170, 170),
    "l_elbow_pitch":    (-150, 15),
    "l_wrist_yaw":      (-170, 170),
    "l_wrist_pitch":    (-45, 60),
    "l_wrist_roll":     (-95, 75),
    # 右臂（注意右肩侧展是 -150~+15）
    "r_shoulder_pitch": (-170, 170),
    "r_shoulder_roll":  (-150, 15),
    "r_shoulder_yaw":   (-170, 170),
    "r_elbow_pitch":    (-150, 15),
    "r_wrist_yaw":      (-170, 170),
    "r_wrist_pitch":    (-45, 60),
    "r_wrist_roll":     (-75, 95),
    # 腿（双侧相同）
    "l_hip_roll":   (-45, 45),
    "l_hip_pitch":  (-160, 120),
    "l_hip_yaw":    (-60, 60),
    "l_knee_pitch": (0, 137),
    "l_ankle_pitch":(-70, 30),
    "l_ankle_roll": (-30, 30),
    "r_hip_roll":   (-45, 45),
    "r_hip_pitch":  (-160, 120),
    "r_hip_yaw":    (-60, 60),
    "r_knee_pitch": (0, 137),
    "r_ankle_pitch":(-70, 30),
    "r_ankle_roll": (-30, 30),
}

# 默认速度(rpm)与电流(A)：如需更严格限制可按关节自定义
DEFAULT_SPD_RPM = 30.0
DEFAULT_CUR_A   = 3.0

class JointStateBridge(object):
    def __init__(self):
        self.pub = {
            "head":  rospy.Publisher("/head/set_pos",  CmdSetMotorPosition, queue_size=10),
            "waist": rospy.Publisher("/waist/set_pos", CmdSetMotorPosition, queue_size=10),
            "arm":   rospy.Publisher("/arm/set_pos",   CmdSetMotorPosition, queue_size=10),
            "leg":   rospy.Publisher("/leg/set_pos",   CmdSetMotorPosition, queue_size=10),
        }
        self.last_cmd = rospy.Time(0)
        self.min_dt   = rospy.Duration(0.02)  # 50 Hz 限速发布，避免刷屏

        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states, queue_size=1)

    def clamp_deg(self, name, deg):
        if name in DEG_LIMITS:
            lo, hi = DEG_LIMITS[name]
            return max(lo, min(hi, deg))
        return deg

    def cb_joint_states(self, msg):
        now = rospy.Time.now()
        if (now - self.last_cmd) < self.min_dt:
            return

        # 将各 group 的命令聚合成数组消息，一次各发一个
        grouped = defaultdict(list)

        for jname, jpos in zip(msg.name, msg.position):
            if jname not in JOINT_MAP:
                continue
            motor_id, group = JOINT_MAP[jname]

            # 弧度 -> 度 做限幅，再转回弧度
            deg = math.degrees(jpos)
            deg = self.clamp_deg(jname, deg)
            pos_rad = math.radians(deg)

            cmd = SetMotorPosition()
            cmd.name = motor_id
            cmd.pos  = pos_rad
            cmd.spd  = DEFAULT_SPD_RPM
            cmd.cur  = DEFAULT_CUR_A
            grouped[group].append(cmd)

        for group, arr in grouped.items():
            if not arr: 
                continue
            out = CmdSetMotorPosition()
            out.cmds = arr
            self.pub[group].publish(out)

        self.last_cmd = now

if __name__ == "__main__":
    rospy.init_node("joint_state_to_tiangong_bridge")
    JointStateBridge()
    rospy.loginfo("joint_state_to_tiangong_bridge started.")
    rospy.spin()
