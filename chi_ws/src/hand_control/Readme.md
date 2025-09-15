# Code using guide 
# 放脚本
```
chmod +x ~/rosws/src/hand_control/scripts/hand_opener.py
cd ~/rosws && catkin_make && source devel/setup.bash
```
# 左手 80% 张开（全手 0..5）
```
roslaunch hand_control hand_open_service.launch side:=left_hand ratio:=0.8
```
# 右手全闭
```
roslaunch hand_control hand_open_service.launch side:=right_hand ratio:=0.0
```
# 自定义每指权重（让拇指相对开小一点）
```
roslaunch hand_control hand_open_service.launch ratio:=1.0 weights:="[1,1,1,1,0.7,0.6]"
```