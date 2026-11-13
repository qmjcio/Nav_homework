编译指令：
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/qmjcio/Nav_homework.git nav_homework
cd ~/ros2_ws
colcon build --packages-select nav_homework
source install/setup.bash
ros2 launch nav_homework nav_homework.launch.py
设计思路：
寻路分成读图---》A*---》发送路径三个步骤，由此诞生了大致三版，此作业采用最后一版：
第一版：
纯C++，利用opencv读图进行路线规划
优点：清晰明确
缺点：不能与ros对接
第二版：
以第一版为基础写出一个类，再额外写一个map_publisher.cpp作为mapserver的替代
第三版：
直接用mapserver进行通讯
