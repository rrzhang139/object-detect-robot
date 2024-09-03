#!/bin/bash
cd ~/arebot_ws_arm
. devel/setup.sh
roslaunch arebot_base arebot_base.launch &
sleep 10
echo "robot base starting success!"

roslaunch szarbot_arm arm.launch &
sleep 15
echo "robot arm starting success!"

roslaunch tag_deal apriltag_bring_up.launch &
sleep 10
echo "robot arm starting success!"

wait
exit 0

