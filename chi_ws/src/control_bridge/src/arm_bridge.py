#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv, time, math
import rospy
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

def to_float(x):
    try:
        return float(x)
    except Exception:
        return None

class CSVToArmSetBridge(object):
    """
    读取 CSV 并按行把轨迹发到 /arm/set（SDK 位置模式）。
    规则：
      - 忽略以 '#' 开头的注释行
      - 第一条非注释行视为表头
      - 时间列：列名包含 'time'（不区分大小写）
      - 位置列：其余列默认视为位置列（单位默认弧度；若 CSV 是度，可设参数转换）
      - 同名可选列：'<pos_col>_vel' (rpm) 和 '<pos_col>_cur' (A)；缺省时用默认参数
      - 关节 ID：
          * 若位置列名本身就是数字（如 '21'），直接用作 `name`
          * 否则用参数 ~joint_map（YAML）提供“列名→ID”的映射
    播放：
      - 默认按 CSV 的 time_from_start(s) 精确等待
      - 若没找到时间列或关闭 use_file_timing，则按 ~rate_hz 匀速播放
    """
    def __init__(self):
        # 必需/常用参数
        self.csv_path         = rospy.get_param("~csv_path")                 # 必填
        self.topic_out        = rospy.get_param("~topic_out", "/arm/cmd_pos")    # SDK 位置模式话题
        self.input_in_degrees = rospy.get_param("~input_in_degrees", False)  # CSV 是否用度
        self.default_spd_rpm  = float(rospy.get_param("~default_spd_rpm", 60.0))
        self.default_cur_A    = float(rospy.get_param("~default_cur_A",   5.0))
        self.use_file_timing  = rospy.get_param("~use_file_timing", True)
        self.rate_hz          = float(rospy.get_param("~rate_hz", 100.0))
        self.skip_bad_rows    = rospy.get_param("~skip_bad_rows", True)
        self.joint_map        = rospy.get_param("~joint_map", {})  # 例如 {shoulder_pitch_r_joint: 21, ...}

        if not self.csv_path:
            rospy.logfatal("~csv_path 不能为空"); raise SystemExit(1)

        self.pub = rospy.Publisher(self.topic_out, CmdSetMotorPosition, queue_size=10)

        # 读 CSV（流式，首行取表头）
        with open(self.csv_path, "r") as f:
            reader = csv.reader(f)
            self.header = None
            self.data_rows = []
            for raw in reader:
                if not raw: continue
                # 处理整行被当成一个单元格的情况
                if len(raw) == 1 and "," in raw[0]:
                    raw = [c.strip() for c in raw[0].split(",")]
                if raw[0].strip().startswith("#"):
                    continue
                if self.header is None:
                    self.header = [c.strip() for c in raw]
                else:
                    if self.skip_bad_rows and any("..." in c for c in raw):
                        rospy.logwarn("跳过含 '...' 的行：%s", raw[:3]); continue
                    self.data_rows.append([c.strip() for c in raw])

        if not self.header or not self.data_rows:
            rospy.logfatal("CSV 无有效数据：%s", self.csv_path); raise SystemExit(1)

        # 列归类
        self.time_idx = None
        self.pos_cols = []  # [(idx, name), ...]  按 CSV 表头顺序
        self.vel_cols = {}  # name -> idx
        self.cur_cols = {}  # name -> idx

        for i, h in enumerate(self.header):
            hlow = h.lower()
            if self.time_idx is None and "time" in hlow:
                self.time_idx = i
                continue

        # 先把所有“明显是速度/电流”的记下来
        for i, h in enumerate(self.header):
            hlow = h.lower()
            if i == self.time_idx: continue
            if hlow.endswith("_vel") or hlow.endswith("_joint_vel"):
                self.vel_cols[h] = i
                continue
            if hlow.endswith("_cur") or hlow.endswith("_joint_cur"):
                self.cur_cols[h] = i
                continue

        # 其余列作为“位置列”
        for i, h in enumerate(self.header):
            if i == self.time_idx: continue
            if h in self.vel_cols or h in self.cur_cols: continue
            self.pos_cols.append((i, h))

        rospy.loginfo("解析完成：rows=%d, time_idx=%s, pos=%d, vel=%d, cur=%d",
                      len(self.data_rows), str(self.time_idx),
                      len(self.pos_cols), len(self.vel_cols), len(self.cur_cols))

    def _name_to_id(self, col_name):
        # 1) 列名本身是数字
        s = str(col_name).strip()
        if s.isdigit():
            return int(s)
        # 2) joint_map 提供映射
        if col_name in self.joint_map:
            return int(self.joint_map[col_name])
        # 3) 简单别名归一
        alias = col_name.replace("_r_", "_right_").replace("_l_", "_left_")
        if alias in self.joint_map:
            return int(self.joint_map[alias])
        return None

    def _to_rad(self, x):
        return math.radians(x) if self.input_in_degrees else x

    def play(self):
        start_wall = time.time()
        first_t = None
        rate = rospy.Rate(self.rate_hz)

        for r_idx, row in enumerate(self.data_rows):
            if rospy.is_shutdown():
                break

            # 根据 time_from_start 等待
            if self.use_file_timing and self.time_idx is not None and self.time_idx < len(row):
                t = to_float(row[self.time_idx])
                if t is not None:
                    if first_t is None: first_t = t
                    target = t - first_t
                    while not rospy.is_shutdown():
                        elapsed = time.time() - start_wall
                        if elapsed + 1e-4 >= target:
                            break
                        time.sleep(min(0.002, max(0.0, target - elapsed)))
            else:
                rate.sleep()

            cmd = CmdSetMotorPosition()

            # 按“CSV 表头顺序”构建 cmds
            for col_idx, col_name in self.pos_cols:
                if col_idx >= len(row): continue
                pos_v = to_float(row[col_idx])
                if pos_v is None: continue

                jid = self._name_to_id(col_name)
                if jid is None:
                    # 列名不映射到ID -> 跳过（避免发错）
                    rospy.logwarn_once("列 '%s' 未映射到电机ID，将被跳过（可在 ~joint_map 中配置）。", col_name)
                    continue

                # 匹配速度/电流列（优先 '<pos>_joint_vel/cur'，其次 '<pos>_vel/cur'）
                vel_v = None; cur_v = None
                for key in (col_name.replace("_joint", "_joint_vel"), col_name + "_vel"):
                    if key in self.vel_cols and self.vel_cols[key] < len(row):
                        vel_v = to_float(row[self.vel_cols[key]])
                        if vel_v is not None: break
                for key in (col_name.replace("_joint", "_joint_cur"), col_name + "_cur"):
                    if key in self.cur_cols and self.cur_cols[key] < len(row):
                        cur_v = to_float(row[self.cur_cols[key]])
                        if cur_v is not None: break

                sp = SetMotorPosition()
                sp.name = int(jid)
                sp.pos  = float(self._to_rad(pos_v))  # 弧度
                sp.spd  = float(vel_v if vel_v is not None else self.default_spd_rpm)  # rpm
                sp.cur  = float(cur_v if cur_v is not None else self.default_cur_A)    # A
                cmd.cmds.append(sp)

            if cmd.cmds:
                self.pub.publish(cmd)
            else:
                rospy.logwarn("第 %d 行没有生成任何有效命令（可能缺少 joint_map 或列名不是ID）。", r_idx)

def main():
    rospy.init_node("csv_to_arm_set_bridge")
    bridge = CSVToArmSetBridge()
    rospy.loginfo("开始播放 CSV：%s", rospy.get_param("~csv_path"))
    bridge.play()
    rospy.loginfo("播放结束。")

if __name__ == "__main__":
    main()
