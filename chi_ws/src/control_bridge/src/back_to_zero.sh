#!/bin/bash
# 控制左右臂所有关节回到 0 rad 位置
# 用法: ./both_arms_home.sh [speed]
# 例如: ./both_arms_home.sh 20   # 速度设为20
#       ./both_arms_home.sh      # 默认速度30

SPEED=${1:-30}   # 如果没传参数，默认30

rostopic pub -1 /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{
  header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''},
  cmds: [
    {name: 11, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 12, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 13, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 14, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 15, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 16, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 17, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 21, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 22, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 23, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 24, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 25, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 26, pos: 0.0, spd: $SPEED, cur: 1.0},
    {name: 27, pos: 0.0, spd: $SPEED, cur: 1.0}
  ]
}"
