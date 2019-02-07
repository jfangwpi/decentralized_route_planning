# Decentralized task assignment and route-planning among multiple autonomous vehicles system

## 1. Development Environment

* OS: Ubuntu 16.04,
* Compiler: gcc 4.7+
* Building System: CMake

## 2. Install Dependencies

# Update system
```
$ sudo apt-get update
$ sudo apt-get -y upgrade
```

# Development tools for C++, java, and python 
```
$ sudo apt-get -y install build-essential git cmake
$ sudo apt-get -y install openjdk-8-jdk
$ sudo apt-get -y install python-dev python-pip
$ sudo apt-get -y install python-numpy python-scipy python-matplotlib
```

# Commonly used libraries 
```
$ sudo apt-get install autoconf
$ sudo apt-get install libglib2.0-dev
$ sudo apt-get -y install autotools-dev automake autopoint libtool
$ sudo apt-get -y install libopencv-dev python-opencv
$ sudo apt-get -y install libboost-all-dev libeigen3-dev
$ sudo apt-get -y install libcgal-dev
```

# Install Visual Studio code (Optional)
Can downloaded from https://code.visualstudio.com/ and installed mannually. Otherwise, run:
```
$ sudo apt-get -y install apt-transport-https 
$ curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
$ sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
$ sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
$ sudo apt-get update
$ sudo apt-get -y install code
```

# Spot (Required by LTL)
Download the latest version of spot from website: https://spot.lrde.epita.fr/install.html
Extract files into ~/software
```
$ cd ~/software
$ ./configure
$ ./configure --disable-python
$ make
$ sudo make install

$ sudo apt-get install gedit
$ gedit ~/.bashrc
Add the following line to your ~/.bashrc
$ export LD_LIBRARY_PATH=:/usr/local/lib/
```


## 3. Set up workspace
Set up the workspace at any location as you prefer. Here I use "~/Workspace/task_assignment" as example
```
$ mkdir -p ~/Workspace/task_assignment
$ cd ~/Workspace/task_assignment
$ git init
$ git remote add origin https://github.com/jfangwpi/decentralized_route_planning.git
$ git pull origin master
```

## 4. Build the project
```
$ mdkir build
$ cd build
$ cmake ../src
$ make
```

## 6. Test the example
### Visualize the map
Edit the configuration files for agents, tasks and map. These configuration files are stored at "/task_assignment/src/config"
Once the editing is completed, run 
```
$ cd bin
$ ./map_vis_demo
```
Check the result at "/task_assignment/build/bin", the result is the figure called "result_map.jpg". Example of the result can be: 
<img src="/data/result_map.jpg" align="middle" height="500" >

#### Comments about result
1. The grey cells are obstacles, orange cells are regions of interest (tasks) and the cell marked by v_i is the initial position of vehicle i.


### Task assignment among multiple vehicles by CBBA (Independent task ONLY)
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment among multiple vehicles by CBBA
```
$ cd bin
$ ./cbba_demo
```
Check the result at "/task_assignment/build/bin", the result is the figure called "result_cbba.jpg". Example of the result can be: 
<img src="/data/result_cbba.jpg" align="middle" height="500" >

#### Comments about result
1. The feasible path for vehicle i is draw by straight line with corresponding color, i.e., vehicle 1 is required to move to cell 6 first, then move to cell 75 along the blue line.  


### Task assignment of both independent and dependent tasks among multiple vehicles
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment of both independent and dependent tasks via proposed method
```
$ cd bin
$ ./syn_demo
```
Check the result at "/task_assignment/build/bin", the resut is the figure called "result_syn.jpg". Example of the result can be:
<img src="/data/result_syn.jpg" align="middle" height="500" >

#### Comments about result
1. In this case, 2 vehicles are required for visiting cell 156 and 3 vehicles are required for visiting cell 84.
2. Base on the result shown above, vehicle 3 will arrive cell 156 first and wait until vehicle 1 and 2 arrive. Then vehicle 1 and 3 will keep moving to cell 84.
3. In this case, the waiting time is 18(unit).


### Task assignment of both independent and dependent tasks among multiple vehicles (CBGA)
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment of both independent and dependent tasks via CBGA
```
$ cd bin
$ ./cbga_demo
```
Check the result at "/task_assignment/build/bin", the resut is the figure called "result_syn.jpg". Example of the result can be:
<img src="/data/result_cbga.jpg" align="middle" height="500" >

#### Comments about result
1. In this case, 2 vehicles are required for visiting cell 156 and 3 vehicles are required for visiting cell 84.
2. Base on the result shown above, vehicle 1 and 2 will move to cell 84 first and then vehicle 2 will move to cell 156 to visit cell 156 with vehicle 3 and 4.
3. In this case, the total waiting time is 18(unit).

### Task assignment with considering kinematic constraints
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment of both independent and dependent tasks via proposed method:
```
$ cd bin
$ ./syn_cbta_demo
```
Check the result at "/task_assignment/build/bin", the resut is the figure called "result_syn_cbta.jpg". Example of the result can be:
<img src="/data/result_syn_cbta_png" align="middle" height="500" >

#### Comments about result
1. In this case, minimum radius of turn is defined as 3(units) for all vehicles in the group.
2. The total waiting time required is 10 (units).


### Task assignment with considering kinematic constraints (CBGA)
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment of both independent and dependent tasks via CBGA:
```
$ cd bin
$ ./cbga_cbta_demo
```
Check the result at "/task_assignment/build/bin", the resut is the figure called "result_cbga_cbta.jpg". Example of the result can be:
<img src="/data/result_cbga_cbta_png" align="middle" height="500" >

#### Comments about result
1. In this case, minimum radius of turn is defined as 3(units) for all vehicles in the group.
2. The total waiting time required is 24 (units).

