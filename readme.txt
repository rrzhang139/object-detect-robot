
//run car 
cd ~/arebot_ws_arm
. devel/setup.sh
roslaunch arebot_base arebot_base.launch 


rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 



//run arm
cd ~/arebot_ws_arm
. devel/setup.sh
roslaunch szarbot_arm arm.launch

rostopic pub /szarbot_arm/suction_cup std_msgs/ol "data: true"
rostopic pub /szarbot_arm/suction_cup std_msgs/Bool "data: false" 


rostopic pub /szarbot_arm/position geometry_msgs/Point "x: 170.0
y: 0.0
z: 220.0"

rostopic pub /szarbot_arm/position geometry_msgPoint "x: 190.0
y: 0.0
z: 220.0" 

rostopic pub /szarbot_arm/position geometry_msgPoint "x: 170.0
y: 0.0
z: 370.0" 



//run vision apriltag
cd ~/arebot_ws
. devel/setup.sh
roslaunch tag_deal apriltag_bring_up.launch


