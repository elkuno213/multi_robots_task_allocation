[Corot Project](http://www.corot-project.org/): Multi Robots Task Allocation (MRTA)
==========

The [Corot Project](http://www.corot-project.org/), funded by the European Commission Interreg France-Channel-England Program, aims to improve the design of flexible and responsive manufacturing systems involving autonomous Collaborative Robots (CoRoT). 

My MRTA project, as a part of the CoRot, is built at the laboratory [LINEACT EA 7527](https://recherche.cesi.fr/) with objective to construct a ROS-Melodic-based system of multiple AGV Robots assembled by MIR100 robot, UR5 robot and Robotiq 2F 140 Gripper. This system is controlled by a supervisor system, which contains a mission planning algorithm


Package overview
----------------

* `mrta`: Algorithm of task allocation to robots
* `mir_robot`: Metapackage for MIR100 robot
* `universal_robot`: Metapackage for UR5 robot
* `manipulator`: Metapackage for the manipulator assembled by UR5 robot and Robotiq 2F 140 gripper
* `mm_robot`: Metapackage for the mobile manipulator assembled from MIR100 robot, UR5 robot and Robotiq 2F 140 Gripper
* `usecase_gazebo`: Environment and map of the usecase for Gazebo simulation


Installation
------------

#### Miniconda environment

* Install Miniconda by following these two links: [Miniconda](https://docs.conda.io/en/latest/miniconda.html), [Instructions](https://conda.io/projects/conda/en/latest/user-guide/install/linux.html)

* Create `MRTA` environment: 
```
conda create -n MRTA python=2.7
```

* Install required packages:
```
conda install -c conda-forge rospkg -n MRTA
conda install -c sotte empy -n MRTA
conda install -c conda-forge defusedxml -n MRTA
conda install -c miniconda numpy -n MRTA
conda install -c miniconda pyqtgraph -n MRTA
conda install -c miniconda yaml -n MRTA
conda install -c miniconda pycryptodomex -n MRTA
conda install -c conda-forge python-gnupg -n MRTA
conda install -c conda-forge matplotlib -n MRTA
```

* Add activation to file `.bashrc` or file `.zshrc`:
```
echo "conda activate MRTA" >> ~/.bashrc
### Or
echo "conda activate MRTA" >> ~/.zshrc
```

* Add shebang to python files:
```
#!/home/hvh/miniconda3/envs/MRTA/bin/python
```

#### ROS workspace

* Install ROS Melodic by following this link: [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

* Creare `MRTA` workspace:
```
### Create MRTA_ws
mkdir ~/MRTA_ws
cd ~/MRTA_ws/

### Clone repository into workspace
git clone https://github.com/huynhvuh/multi_robots_task_allocation.git
chmod -R a+rX *

### Use rosdep to install all dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y

### Source-install of "assimp" package to fix error of "failed to import assimp" of "moveit_commander"
conda activate MRTA
git clone https://github.com/assimp/assimp.git
cd ./assimp
mkdir build && cd build
cmake .. -G 'Unix Makefiles'
make -j4
cd ..
cd ./port/PyAssimp
python setup.py install

### Build workspace
cd ~/MRTA_ws/
catkin_make

### Echo source to ".bashrc" file
echo "source ~/MRTA_ws/devel/setup.bash" >> ~/.bashrc
### Or to ".zshrc" file
echo "source ~/MRTA_ws/devel/setup.zsh" >> ~/.zshrc
```

Multi-PC configuration
------------

* On the master PC: 
Run `hostname -I` to get the IP address (Ex: 10.191.76.93), then add it to the source file:
```
### Echo to ".bashrc" file
echo "export ROS_IP=10.191.76.93" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://10.191.76.93:11311" >> ~/.bashrc
### Or to ".zshrc" file
echo "export ROS_IP=10.191.76.93" >> ~/.zshrc
echo "export ROS_MASTER_URI=http://10.191.76.93:11311" >> ~/.zshrc
```

* On the other PCs:
Run `hostname -I` to get the IP address (Ex: 10.191.76.59), then add it to the source file:
```
### Echo to ".bashrc" file
echo "export ROS_IP=10.191.76.59" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://10.191.76.93:11311" >> ~/.bashrc
### Or to ".zshrc" file
echo "export ROS_IP=10.191.76.59" >> ~/.zshrc
echo "export ROS_MASTER_URI=http://10.191.76.93:11311" >> ~/.zshrc
```


Launch simulation
------------
Each of the following command-lines should be run on each PC to maximize the simulation time (but not mandatory). Below is an example of 2 robots:
```
### Run the simulated usecase
roslaunch usecase_gazebo usecase_world_for_multi_mms.launch paused:="false"  

### Run 2 AGV robots
roslaunch multi_mms indivi_mm.launch robot_name:="robot0" pose_x:="1" pose_y:="1"
roslaunch multi_mms indivi_mm.launch robot_name:="robot`" pose_x:="1" pose_y:="2"

### Run mission planning algorithm (2 products to manufacture)
python ~/MRTA_ws/src/mrta/scripts/MRTA_algorithm.py 
```


Fix potential errors
------------

* Error `[Err] [REST.cc:205] Error in REST request`:
Inside the file `.ignition/fuel/config.yaml`, replace `url: https://api.ignitionfuel.org` by `url: https://api.ignitionrobotics.org`
