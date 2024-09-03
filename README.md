# Object detection and manipulation robot using Jetson Xavier NX GPU


## Setup

#### Start ROS
// Open new Terminal 
```
roscore
```

### Compile workspace and configure env, launch base robot
// Open new terminal
```
catkin_make
source devel/setup.sh 
roslaunch arebot_base arebot_base.launch 
```

#### Launch main package
```
//Open new terminal
source devel/setup.sh
roslaunch detection detection.launch
```



