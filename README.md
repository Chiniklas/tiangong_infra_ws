This branch is for doing real experiments on the tiangong 2 pro robots.

**Section 1: Hardware initialization guidance**

**1.1 Power up**

At the waist of the robot, at the back, there are a line of buttons and slots. To power up the robot, you need to press once the big button on the very left, the button will turn green. In the meantime, the button on the very right should be red because the emergency stop button is pressed down. Now release the emergency stop button and press the smaller button on the very right, the cooling vans will start to work and the button will turn green after 6 second. 

Explanation: the left button is for the whole body power supply, and the right button is for starting up the micro computer on the robot, so there is a sequence of starting up.

**1.2 Connection to the host machine.**

**1.2.1 Wired connection (Ethernet)**

To connect to the robot and perform tests, you need a computer with linux systems (preferably Ubuntu 20.04, for running ROS1 nodes) and ethernet connection. It is recommended to connect your computer with the robot using a ethernet cable longer than 2 meters. Then set your cable connection to 192.168.41.XXX (XXX must avoid 1,2,3, these addresses are already taken by the robot). I'll take 192.168.41.108 as an example, and set the netmask to 255.255.255.0.

<img width="984" height="830" alt="image" src="https://github.com/user-attachments/assets/4da13e3c-9412-4fa8-827e-66576fffdd45" />

**1.2.2 Wireless connection (WLAN)**

Currently there is a local network available called AIRhumanoid, the password is the same as the name.

**1.3 Login to remote machine**

If your network setting is successful, you should be able to ping the robot(the address is 192.168.41.1). Try: **ping 192.168.41.1**. If you are using wireless connection, you should be able to find the address of the robot using **ifconfig**. 

Now you should be able to login to the remote machine using **ssh ubuntu@192.168.41.1**, the password is **tg2TUM2025** Then you can see your terminal is linked to the remote machine.

**Section 2: Control of the robot**

**2.1 Launch ROS control node**

ROS1 is prebuilt and added to the environment path beforehand, so you should be able to run **roscore** directly on your terminal. Open another terminal linking to the remote machine following step 1.3, first give it administrative right by running **sudo su**, the password is the same as ssh password. The terminal should have a root label on it. 

Now go to **rosws** on the remote machine, run **source install_isolated/setup.bash**. Finally, run **roslaunch body_control body.launch**. You should see all the control apis appear if you call **rostopic list** and **rosservice list**. The robot will reset to its preset zero position with high Kp and Kd. Now you are ready for testing.

**2.2 Run this demo repo**

To run the demos in this repo, you should either **git clone** it directly to the remote machine on the robot or move it from host machine to the remote machine using **scp** commands. It is highly recommended that you rebuild the ros packages on the root directory by running **rm -rf /build /devel** and **catkin_make**. Then source the build as well, by **source chi_ws/devel/setup.bash**. Now you should be able to find our customized packages using **roscd**.

**Section 3: Power down**

After your tests, it is very important to power down the robot and put it in a safe place. Because the battery is unstable under puncture attack and the it could cause explosion and fire hazard if accidentally damaged. Please first press emergency stop and then power button, the cooling fans will stop automatically. Then hang it up using the crane properly to avoid collision, now you are free to go.

**Common problems**

1- if the robot is beeping after powering up with the light on the lower chest blinking red, it means the robot needs recharging. Don't leave it unwatched while charging and 3-4 hours is sufficient. If you need to continue testing, 2 hours charging is totally acceptable. **DO NOT LEAVE IT CHARGING OVERNIGHT!** You could always charge during the day which people are around.

![261c399327deb0ceeb0e4efbed178160](https://github.com/user-attachments/assets/b33706df-5575-47dd-a814-63d852c96890)

