#!/bin/bash
# 播放 CSV 轨迹，逐行发布到 /arm/cmd_pos
# 用法:
#   ./play_traj.sh             # 默认 spd=30
#   ./play_traj.sh 50          # spd=50

CSV_FILE="traj_csv.csv"

SPD=${1:-30}   # 从命令行参数读取速度，没给就用30

last_t=0.0

# 读取 CSV，跳过注释(#)和表头
tail -n +8 "$CSV_FILE" | while IFS=, read -r t q1 q2 q3 q4 q5 q6 q7 rest; do
  # 计算相对 sleep 时间
  dt=$(echo "$t - $last_t" | bc -l)
  if (( $(echo "$dt > 0" | bc -l) )); then
    sleep $dt
  fi
  last_t=$t

  # 发布指令
  rostopic pub -1 /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{
    header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''},
    cmds: [
      {name: 21, pos: $q1, spd: $SPD, cur: 1.0},
      {name: 22, pos: $q2, spd: $SPD, cur: 1.0},
      {name: 23, pos: $q3, spd: $SPD, cur: 1.0},
      {name: 24, pos: $q4, spd: $SPD, cur: 1.0},
      {name: 25, pos: $q5, spd: $SPD, cur: 1.0},
      {name: 26, pos: $q6, spd: $SPD, cur: 1.0},
      {name: 27, pos: $q7, spd: $SPD, cur: 1.0}
    ]
  }"
done
