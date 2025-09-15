#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import math
import rospy
import moveit_commander
import geometry_msgs.msg
import visualization_msgs.msg

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

# ---------------- tf2 工具 ----------------
def tf2_transform(ps_in, target_frame, tf_timeout=1.0):
    """把 PoseStamped 从其 header.frame_id 变换到 target_frame"""
    buf = tf2_ros.Buffer(rospy.Duration(5.0))
    _ = tf2_ros.TransformListener(buf)
    buf.can_transform(target_frame, ps_in.header.frame_id, rospy.Time(0), rospy.Duration(tf_timeout))
    tf = buf.lookup_transform(target_frame, ps_in.header.frame_id, rospy.Time(0), rospy.Duration(tf_timeout))
    return tf2_geometry_msgs.do_transform_pose(ps_in, tf)

# ---------------- 主功能 ----------------
def go_to_point(
    group_name="left_arm",
    src_frame="base",
    xyz=(0.3, 0.2, 0.4),
    rpy=None,
    planner="ompl",
    cartesian_step=0.02,
    ompl_id=None,
    planning_time=10.0,
    planning_attempts=50,
    marker_hold_sec=2.0
):
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    group = moveit_commander.MoveGroupCommander(group_name)
    marker_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker, queue_size=1, latch=True)

    # ---- 配置规划器 ----
    if planner == "ompl":
        group.set_planner_id(ompl_id or "RRTConnectkConfigDefault")
        group.set_planning_time(planning_time)
        group.set_num_planning_attempts(planning_attempts)
    elif planner == "pilz_lin":
        group.set_planner_id("LIN")
        group.set_planning_time(3.0)
    elif planner == "cartesian":
        pass
    else:
        rospy.logwarn("未知 planner %s，回落 OMPL", planner)
        group.set_planner_id("RRTConnectkConfigDefault")

    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_goal_tolerance(1e-3)

    planning_frame = group.get_planning_frame()
    rospy.loginfo("Planning frame: %s", planning_frame)

    # ---- 目标姿态 ----
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = src_frame
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz

    if rpy is None:
        ps.pose.orientation = group.get_current_pose().pose.orientation
    else:
        qx, qy, qz, qw = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz; ps.pose.orientation.w = qw

    ps_plan = ps if src_frame == planning_frame else tf2_transform(ps, planning_frame)

    rospy.loginfo("目标点: (%.3f, %.3f, %.3f)", ps_plan.pose.position.x, ps_plan.pose.position.y, ps_plan.pose.position.z)
    rospy.loginfo("目标姿态 quaternion: [%.3f, %.3f, %.3f, %.3f]",
                  ps_plan.pose.orientation.x, ps_plan.pose.orientation.y,
                  ps_plan.pose.orientation.z, ps_plan.pose.orientation.w)

    if rpy is not None:
        rospy.loginfo("目标姿态 RPY: roll=%.2f°, pitch=%.2f°, yaw=%.2f°",
                      math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2]))

    # ---- RViz 标注 ----
    for _ in range(30):
        if marker_pub.get_num_connections() > 0:
            break
        rospy.sleep(0.1)

    # 球体
    m = visualization_msgs.msg.Marker()
    m.header.frame_id = planning_frame
    m.header.stamp = rospy.Time.now()
    m.ns = "tg2_point_demo"
    m.id = 1
    m.type = visualization_msgs.msg.Marker.SPHERE
    m.action = visualization_msgs.msg.Marker.ADD
    m.pose = ps_plan.pose
    m.scale.x = m.scale.y = m.scale.z = 0.04
    m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.9
    marker_pub.publish(m)

    # 文本
    txt = visualization_msgs.msg.Marker()
    txt.header.frame_id = planning_frame
    txt.header.stamp = rospy.Time.now()
    txt.ns = "tg2_point_demo"
    txt.id = 2
    txt.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
    txt.action = visualization_msgs.msg.Marker.ADD
    txt.pose = copy.deepcopy(ps_plan.pose)
    txt.pose.position.z += 0.06
    txt.scale.z = 0.06
    txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 0.9
    txt.text = "Target"
    marker_pub.publish(txt)

    # 姿态箭头
    arrow = visualization_msgs.msg.Marker()
    arrow.header.frame_id = planning_frame
    arrow.header.stamp = rospy.Time.now()
    arrow.ns = "tg2_point_demo"
    arrow.id = 3
    arrow.type = visualization_msgs.msg.Marker.ARROW
    arrow.action = visualization_msgs.msg.Marker.ADD
    arrow.pose = ps_plan.pose
    arrow.scale.x = 0.15  # 箭长
    arrow.scale.y = 0.02  # 箭宽
    arrow.scale.z = 0.02
    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.1, 0.1, 0.9
    marker_pub.publish(arrow)

    # ---- 规划逻辑 ----
    group.set_start_state_to_current_state()

    # A 精确姿态
    group.set_goal_orientation_tolerance(1e-3)
    group.set_pose_target(ps_plan)
    planA = group.plan()
    jtA = getattr(planA, "joint_trajectory", None) or (isinstance(planA, tuple) and getattr(planA[1], "joint_trajectory", None))
    if jtA and jtA.points:
        rospy.loginfo("使用精确姿态规划成功，执行…")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        return

    # B 放宽姿态
    rospy.logwarn("姿态目标精确解失败，尝试放宽至 10°")
    group.clear_pose_targets()
    group.set_goal_orientation_tolerance(math.radians(10))
    group.set_pose_target(ps_plan)
    planB = group.plan()
    jtB = getattr(planB, "joint_trajectory", None) or (isinstance(planB, tuple) and getattr(planB[1], "joint_trajectory", None))
    if jtB and jtB.points:
        rospy.loginfo("使用放宽姿态容差规划成功，执行…")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        return

    # C 仅位置
    rospy.logwarn("放宽姿态也失败，降级为仅位置规划")
    group.clear_pose_targets()
    ps_pos_only = copy.deepcopy(ps_plan)
    ps_pos_only.pose.orientation = group.get_current_pose().pose.orientation
    group.set_pose_target(ps_pos_only)
    planC = group.plan()
    jtC = getattr(planC, "joint_trajectory", None) or (isinstance(planC, tuple) and getattr(planC[1], "joint_trajectory", None))
    if jtC and jtC.points:
        rospy.loginfo("仅位置规划成功，执行…")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        rospy.logerr("仅位置规划也失败：请调整目标点或避开碰撞")

    rospy.sleep(marker_hold_sec)
    moveit_commander.roscpp_shutdown()

# ---------------- 入口 ----------------
if __name__ == "__main__":
    rospy.init_node("arm_point_demo", anonymous=True)

    group_name = rospy.get_param("~group_name", "left_arm")
    src_frame  = rospy.get_param("~src_frame",  "base")
    planner    = rospy.get_param("~planner",    "ompl")
    xyz        = rospy.get_param("~xyz",        [0.3, 0.2, 0.4])
    rpy_param  = rospy.get_param("~rpy",        None)
    rpy        = tuple(rpy_param) if isinstance(rpy_param, (list, tuple)) else None
    step       = float(rospy.get_param("~cartesian_step", 0.02))
    ompl_id    = rospy.get_param("~ompl_id",    None)
    plan_time  = float(rospy.get_param("~planning_time", 2.0))
    plan_atts  = int(rospy.get_param("~planning_attempts", 50))
    marker_hold= float(rospy.get_param("~marker_hold_sec", 2.0))

    go_to_point(
        group_name=group_name,
        src_frame=src_frame,
        xyz=tuple(xyz),
        rpy=rpy,
        planner=planner,
        cartesian_step=step,
        ompl_id=ompl_id,
        planning_time=plan_time,
        planning_attempts=plan_atts,
        marker_hold_sec=marker_hold
    )
