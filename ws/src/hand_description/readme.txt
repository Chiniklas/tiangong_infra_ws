一、模型简介：本模型为Inspire Robots手产品模型，在Solidworks中进行建模，使用sw2urdf插件将模型导出为urdf格式，再通过手工修改，产生并联结构，并使之可以在ros-gazebo环境中进行仿真实验。

二、运行环境：Win10 | Solidworks2021 | Ubuntu20.04.1 | ROS1-noetic | gazebo11.11.0

三、运行方法：
  在Rviz中查看模型：roslaunch display.launch；
  在gazebo中查看模型：roslaunch mylaunch.launch来在gazebo中查看模型；
  在gazebo中驱动模型：
     在>>主目录中打开另一个终端，执行如下命令，其中data后的内容为希望该关节运动到的绝对位置，swiveljoint1_position_controller为控制器名称，修改为其它控制器即可控制其它关节。执行后，即可看到对应关节运动到指定位置。至此，模型构建成功！
      如下为6个控制器的驱动代码:

      rostopic pub -1 /swivel/swiveljoint1_position_controller/command std_msgs/Float64 "data: 0.005"
      rostopic pub -1 /swivel/swiveljoint2_position_controller/command std_msgs/Float64 "data: 0.005"
      rostopic pub -1 /swivel/swiveljoint3_position_controller/command std_msgs/Float64 "data: 0.005"
      rostopic pub -1 /swivel/swiveljoint4_position_controller/command std_msgs/Float64 "data: 0.005"
      rostopic pub -1 /swivel/swiveljoint5_position_controller/command std_msgs/Float64 "data: 0.005"
      rostopic pub -1 /swivel/swiveljoint6_position_controller/command std_msgs/Float64 "data: 0.005"
      (date取值范围0-0.01)
四、注意事项
	1.本urdf文件仅能被gazebo正确读取，在其他平台打开将丢失并联结构（包括Rviz）。
	2.进行gazebo仿真时，建议在左侧Physics菜单中关闭重力，重力会严重影响模型运动。
	3.除大拇指外，其余四指仅有一个主动关节（驱动器），其他均为从动关节；大拇指包括两个驱动器，对应两个主动关节。

5- for a simple one-line controller launch, you can do this:

roslaunch handleft9183 swivel_controller.launch pos1:=0.008 pos2:=0.008 pos3:=0.008 pos4:=0.001 pos5:=0.001 pos6:=0.003

it will autumatically run the rostopic pub job that is mentioned in point3.

