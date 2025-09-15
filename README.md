Instructions:

To build the docker image: currently Ubuntu20.04 + ROS noetic:

step1: Build the image
  
  go to ./scripts, run **build.bash**

<<<<<<< HEAD
step2: Build the container
  
  go to ./scripts, run **run.bash**

Step3: initial setup

  not you should be in the container,go to ws/scripts, run **bootstrap_ws.bash**

Step4: sanity check

  now you should be able to run roscore, and run rviz
=======
step2:
  build the container, go to ./scripts, run **run.bash**


### If running build.bash get the bug (docker: 'compose' is not a docker command.)
 ```
 
sudo apt-get update
sudo apt-get install -y docker-compose-plugin

# test 
docker compose version
#if could not find 
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release; echo $VERSION_CODENAME) stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-compose-plugin

 ```
>>>>>>> de6d8eb5d2413f9264c24e8c0b276e4c55b8a2bc



# pull down this branch 
```
sudo apt update
sudo apt install -y git git-lfs
git lfs install
*********
git clone git@github.com:Chiniklas/tiangong_infra_ws.git
cd tiangong_infra_ws
git checkout tiangong_infra_ws_sim

**********
if you already clone this repo 
cd tiangong_infra_ws
git fetch origin
git checkout tiangong_infra_ws_sim
***********
# only once 
git lfs install

# pull lfs 
git lfs pull
# verification 
git lfs ls-files | grep -i '\.stl'
file ws/tg2_ws/src/tg2_description/urdf/meshes/ankle_pitch_l_link.STL

#  “Stereo lithography” or “Binary STL”，not ASCII 

```