Instructions:

To build the docker image: currently Ubuntu20.04 + ROS noetic:

step1: Build the image
  
  go to ./scripts, run **build.bash**

step2: Build the container
  
  go to ./scripts, run **run.bash**

Step3: initial setup

  not you should be in the container,go to ws/scripts, run **bootstrap_ws.bash**

Step4: sanity check

  now you should be able to run roscore, and run rviz
