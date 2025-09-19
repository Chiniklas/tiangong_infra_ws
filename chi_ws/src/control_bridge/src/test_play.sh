#!/bin/bash
# 播放 CSV 轨迹 -> /arm/cmd_pos（右臂 7 关节）
# 间隔控制：
#   1) 默认跟随 CSV 时间戳
#   2) 设 FIXED_DT 覆盖为固定间隔(秒)，例如 FIXED_DT=0.02
#   3) 设 MAX_DT 限制最长等待(秒)，例如 MAX_DT=0.1
#
# 用法示例：
#   ./play_traj.sh                       # 跟随CSV
#   FIXED_DT=0.02 ./play_traj.sh         # 每帧固定 20ms
#   MAX_DT=0.1 ./play_traj.sh            # 按CSV，但最长只等0.1s
#   SPD=50 CUR=0.8 ./play_traj.sh        # 改速度、电流

CSV_FILE="${CSV_FILE:-traj_csv.csv}"

# 速度/电流参数
SPD="${SPD:-30}"     # rpm
CUR="${CUR:-1.0}"    # A

# 间隔控制参数
FIXED_DT="${FIXED_DT:-}"   # 若设定则使用固定间隔(秒)，例如 0.02
MAX_DT="${MAX_DT:-}"       # 若设定则将等待时间上限夹紧(秒)，例如 0.1

# 电机ID（右臂 21..27，如不同请改）
J1=21; J2=22; J3=23; J4=24; J5=25; J6=26; J7=27

[ -f "$CSV_FILE" ] || { echo "找不到CSV: $CSV_FILE"; exit 1; }

last_t=
# 跳过注释/表头，从第8行开始读：t,q1..q7
tail -n +8 "$CSV_FILE" | while IFS=, read -r t q1 q2 q3 q4 q5 q6 q7 rest; do
  # 第一帧不等待；后续帧根据 FIXED_DT/CSV/上限 计算等待时间
  if [ -z "$last_t" ]; then
    last_t="$t"
  else
    if [ -n "$FIXED_DT" ]; then
      dt="$FIXED_DT"
    else
      dt=$(echo "$t - $last_t" | bc -l)
      # 负值保护
      if (( $(echo "$dt < 0" | bc -l) )); then dt=0; fi
      # 上限夹紧
      if [ -n "$MAX_DT" ] && (( $(echo "$dt > $MAX_DT" | bc -l) )); then
        dt="$MAX_DT"
      fi
    fi
    # 等待（支持小数秒）
    if (( $(echo "$dt > 0" | bc -l) )); then
      sleep "$dt"
    fi
    last_t="$t"
  fi

  # 发送一帧
  rostopic pub -1 /arm/cmd_pos bodyctrl_msgs/CmdSetMotorPosition "{
    header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''},
    cmds: [
      {name: $J1, pos: $q1, spd: $SPD, cur: $CUR},
      {name: $J2, pos: $q2, spd: $SPD, cur: $CUR},
      {name: $J3, pos: $q3, spd: $SPD, cur: $CUR},
      {name: $J4, pos: $q4, spd: $SPD, cur: $CUR},
      {name: $J5, pos: $q5, spd: $SPD, cur: $CUR},
      {name: $J6, pos: $q6, spd: $SPD, cur: $CUR},
      {name: $J7, pos: $q7, spd: $SPD, cur: $CUR}
    ]
  }"
done
