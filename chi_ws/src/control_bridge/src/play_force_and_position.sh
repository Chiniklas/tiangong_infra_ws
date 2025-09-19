#!/bin/bash
# 使用力位混合控制 (CmdMotorCtrl) 播放右臂轨迹（根据 traj_csv.csv）
# 话题: /arm/cmd_ctrl
# 用法示例:
#   ./play_right_arm_ctrl.sh                 # 默认 KP=200, KD=5, SPD=30, TOR=0
#   KP=300 KD=6 SPD=20 TOR=0.0 ./play_right_arm_ctrl.sh

CSV_FILE="${CSV_FILE:-traj_csv.csv}"

# ===== 可调参数（也可在运行时通过环境变量覆盖） =====
KP="${KP:-10}"      # 位置环增益
KD="${KD:-1.0}"        # 速度环增益
SPD="${SPD:-1}"     # 速度上限 (rpm) —— 注意是 rpm
TOR="${TOR:-0.0}"    # 前馈力矩/电流（按接口定义 tor 字段，缺省 0）
# 右臂关节 ID（如你的实际 ID 不同，请在此处修改）
J1=21; J2=22; J3=23; J4=24; J5=25; J6=26; J7=27

# ===== 开始 =====
if [ ! -f "$CSV_FILE" ]; then
  echo "未找到 CSV 文件: $CSV_FILE"
  exit 1
fi

echo "CSV: $CSV_FILE"
echo "KP=$KP KD=$KD SPD=$SPD TOR=$TOR"
echo "右臂关节ID: $J1,$J2,$J3,$J4,$J5,$J6,$J7"
echo "3 秒后开始发送到 /arm/cmd_ctrl (Ctrl+C 可中止)…"
sleep 3

# last_t 用于计算相邻行的时间差 sleep
last_t=0.0
first_data_line_seen=0

# 逐行读取:
# - 跳过以 # 开头的注释
# - 跳过表头（包含 time_from_start 字样）
# - 读取: t, q1..q7（忽略后续速度/加速度列）
# 说明：你的 CSV 第 7 行是表头，后面才是数据；这里用通用判断而非固定行号。
while IFS= read -r line; do
  # 跳过注释
  if [[ "$line" =~ ^# ]]; then
    continue
  fi
  # 跳过空行
  if [[ -z "$line" ]]; then
    continue
  fi
  # 跳过表头
  if [[ "$first_data_line_seen" -eq 0 ]]; then
    if echo "$line" | grep -qi "time_from_start"; then
      first_data_line_seen=1
      continue
    fi
  fi

  # 取前 8 个逗号分隔字段: t, q1..q7
  t=$(echo "$line" | awk -F, '{printf "%s",$1}')
  q1=$(echo "$line" | awk -F, '{printf "%s",$2}')
  q2=$(echo "$line" | awk -F, '{printf "%s",$3}')
  q3=$(echo "$line" | awk -F, '{printf "%s",$4}')
  q4=$(echo "$line" | awk -F, '{printf "%s",$5}')
  q5=$(echo "$line" | awk -F, '{printf "%s",$6}')
  q6=$(echo "$line" | awk -F, '{printf "%s",$7}')
  q7=$(echo "$line" | awk -F, '{printf "%s",$8}')

  # 基本校验：若某个字段为空，则跳过该行
  if [[ -z "$t" || -z "$q1" || -z "$q2" || -z "$q3" || -z "$q4" || -z "$q5" || -z "$q6" || -z "$q7" ]]; then
    echo "跳过异常行: $line"
    continue
  fi

  # 睡眠 dt = t - last_t (允许小数)
  dt=$(echo "$t - $last_t" | bc -l)
  # 若 dt < 0，置 0 以避免负 sleep（容错）
  if (( $(echo "$dt < 0" | bc -l) )); then
    dt=0
  fi
  # GNU coreutils 的 sleep 支持浮点数
  sleep "$dt"
  last_t="$t"

  # 发送一帧：一次将 7 个关节写入 cmds[]
  rostopic pub -1 /arm/cmd_ctrl bodyctrl_msgs/CmdMotorCtrl "{
    header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''},
    cmds: [
      {name: $J1, kp: $KP, kd: $KD, pos: $q1, spd: $SPD, tor: $TOR},
      {name: $J2, kp: $KP, kd: $KD, pos: $q2, spd: $SPD, tor: $TOR},
      {name: $J3, kp: $KP, kd: $KD, pos: $q3, spd: $SPD, tor: $TOR},
      {name: $J4, kp: $KP, kd: $KD, pos: $q4, spd: $SPD, tor: $TOR},
      {name: $J5, kp: $KP, kd: $KD, pos: $q5, spd: $SPD, tor: $TOR},
      {name: $J6, kp: $KP, kd: $KD, pos: $q6, spd: $SPD, tor: $TOR},
      {name: $J7, kp: $KP, kd: $KD, pos: $q7, spd: $SPD, tor: $TOR}
    ]
  }"

  echo "t=${t}s 已发一帧 (pos: $q1,$q2,$q3,$q4,$q5,$q6,$q7)"
done < "$CSV_FILE"

echo "播放完成。"
