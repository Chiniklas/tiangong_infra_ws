**Instructions:**

To build the docker image: currently Ubuntu20.04 + ROS noetic:

**step1: Build the image**
  
  go to ./scripts, run **build.bash**

**step2: Build the container**
  
  go to ./scripts, run **run.bash**

**Step3: initial setup**

  not you should be in the container,go to ws/scripts, run **bootstrap_ws.bash**

**Step4: sanity check**

  now you should be able to run roscore, and run rviz

=============================================================================================


**To run gazebo simulation and swivevel controller**

**Step 1: build ros packages**

  go to tiangong_infra_ws folder on your container, you should see src and scripts folders only. run **catkin_make** on this level. you can see there are build and devel folders poping up.

**Step 2: source current ros build**

  run **source devel/setup.bash**, you should be able to see the ros package **handleft9183**.

**Step 3: test rviz visualization**

  run **roslaunch handleft9183 display.launch**, you should see rviz pop up and the left hand on it. you can ctrl + C to exit once you are done.

**Step 4: test gazebo visualization**

  run **roslaunch handleft9183 mylaunch.launch**, you should be able to see gazebo pop up and the left hand on it.

**Step 5: test swivel controller**

  run **roslaunch handleft9183 swivel_controller.launch** in another terminal, you should be able to see the hand in gazebo hold a fist (default setting).

===============================================================================================

TODO:
  1- sim-to-sim: link the current ros env to mujoco.

  2- sim-to-sim: link the current ros env to isaacsim and isaaclab for future reinforcementlearning

  3- sim-to-real: build the link to real hardware sdk and test on real robot

  4- motion planning for the whole body: test whole body urdf visualization and moveit motion planning.
