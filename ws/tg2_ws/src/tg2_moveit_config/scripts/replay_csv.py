#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

def load_csv(csv_path):
    """
    读取 CSV，返回 (joint_names, list_of_points)
    其中 list_of_points 元素为 (t, positions_dict)
      - t: float, 秒
      - positions_dict: {joint_name: position}
    要求表头至少包含：time_from_start(s) + 关节列
    允许额外列（速度/加速度等），会被忽略。
    """
    if not os.path.isfile(csv_path):
        raise FileNotFoundError("CSV 不存在: %s" % csv_path)

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        rows = [r for r in reader if r and not r[0].startswith("#")]

    if not rows:
        raise ValueError("CSV 空文件: %s" % csv_path)

    header = rows[0]
    # 找时间列（兼容不同写法）
    time_idx = None
    for i, h in enumerate(header):
        hh = h.strip().lower()
        if "time_from_start" in hh or hh in ("t", "time", "time(s)"):
            time_idx = i
            break
    if time_idx is None:
        raise ValueError("未找到时间列（需包含 'time_from_start(s)' 或 'time'）")

    # 推断纯“位置”关节列范围：
    # 我们假定：时间列之后紧跟的是关节位置列；若后面出现 *_vel / *_acc 列则停止。
    joint_names = []
    for j in range(time_idx + 1, len(header)):
        name = header[j].strip()
        low = name.lower()
        if low.endswith("_vel") or low.endswith("_acc"):
            break
        joint_names.append(name)
    if not joint_names:
        raise ValueError("未找到任何关节列，请确认表头。")

    # 读取数据
    pts = []
    for r in rows[1:]:
        if len(r) <= time_idx:
            continue
        try:
            t = float(r[time_idx])
        except Exception:
            continue
        pos_map = {}
        for k, jn in enumerate(joint_names):
            idx = time_idx + 1 + k
            if idx < len(r) and r[idx] != "":
                try:
                    pos_map[jn] = float(r[idx])
                except Exception:
                    pass
        pts.append((t, pos_map))

    # 过滤无效
    pts = [p for p in pts if p[1]]
    if not pts:
        raise ValueError("CSV 中没有有效数据行。")

    # 归一化时间（确保从 0 开始，且严格递增）
    t0 = pts[0][0]
    norm = []
    last_t = -1e9
    for t, pm in pts:
        tt = max(0.0, t - t0)
        if tt <= last_t:  # 强制递增
            tt = last_t + 1e-6
        last_t = tt
        norm.append((tt, pm))

    return joint_names, norm


def make_robot_traj(group, joint_names, points, time_scale=1.0, align_to_current=True):
    """
    将 (joint_names, points) 构造成 RobotTrajectory。
    - points: list of (t, pos_map)
    - time_scale: 时间缩放（>1 放慢，<1 加速）
    - align_to_current: 若当前位姿与第一点偏差很大，则在 t=0 插入当前点方便衔接
    """
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = list(joint_names)

    # 当前关节（用于对齐/填补缺失关节）
    current_positions = dict(zip(group.get_active_joints(), group.get_current_joint_values()))

    # 构造 JointTrajectoryPoint
    def build_point(t, pos_map):
        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration.from_sec(max(0.0, t * max(1e-9, time_scale)))
        # 位置依 joint_names 顺序
        arr = []
        for jn in joint_names:
            if jn in pos_map:
                arr.append(pos_map[jn])
            else:
                # 如果该列缺失，用当前值兜底
                arr.append(current_positions.get(jn, 0.0))
        p.positions = arr
        return p

    pts_msg = [build_point(t, pm) for t, pm in points]

    # 可选：在 t=0 预置当前关节，帮助衔接
    if align_to_current and pts_msg:
        first = pts_msg[0]
        # 计算偏差范数
        cur_vec = [current_positions.get(j, 0.0) for j in joint_names]
        diff = math.sqrt(sum((a - b) ** 2 for a, b in zip(first.positions, cur_vec)))
        if diff > math.radians(3.0):  # 超过 3 度阈值就插入当前点
            p0 = JointTrajectoryPoint()
            p0.time_from_start = rospy.Duration(0.0)
            p0.positions = cur_vec
            pts_msg.insert(0, p0)

    traj.joint_trajectory.points = pts_msg
    return traj


def main():
    rospy.init_node("replay_csv", anonymous=True)

    # 参数
    group_name = rospy.get_param("~group_name", "left_arm")
    csv_path   = rospy.get_param("~csv_path",
                    "/tiangong_infra_ws/ws/tg2_ws/src/tg2_moveit_config/scripts/traj_planned.csv")
    time_scale = float(rospy.get_param("~time_scale", 1.0))    # 0.5=放慢一半；2.0=变慢两倍（更慢）
    align_curr = bool(rospy.get_param("~align_to_current", True))

    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(group_name)

    rospy.loginfo("加载 CSV: %s", csv_path)
    joint_names, points = load_csv(csv_path)
    rospy.loginfo("CSV 关节数: %d, 轨迹点数: %d", len(joint_names), len(points))
    rospy.loginfo("CSV 关节列表: %s", ", ".join(joint_names))

    traj = make_robot_traj(group, joint_names, points,
                           time_scale=time_scale, align_to_current=align_curr)

    # 执行
    rospy.loginfo("开始回放（time_scale=%.3f, align_to_current=%s）…", time_scale, align_curr)
    ok = group.execute(traj, wait=True)
    group.stop()

    if ok:
        rospy.loginfo("回放完成。")
    else:
        rospy.logwarn("回放未成功（group.execute 返回 False）。")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
