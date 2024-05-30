#!/bin/bash

# ~/.bashrc 파일에 설정 추가
echo 'export ROS_MASTER_URI=http://172.30.1.57:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc

echo "alias agv='ssh -X er@172.30.1.79'" >> ~/.bashrc
echo "alias arm='ssh -X er@172.30.1.43'" >> ~/.bashrc

echo "alias golf_code='cd /home/sang/catkin_ws/src/golf_bot && code .'" >> ~/.bashrc
echo "alias golf='roslaunch myagv_controller golf_bot_test.launch'" >> ~/.bashrc

# ~/.bashrc 파일 적용
source ~/.bashrc
