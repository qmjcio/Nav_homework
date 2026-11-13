编译指令：
colcon build --packages-select nav_homework --symlink-install
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
优点：不需要额外下载插件，用costmap库的原生工具即可实现
缺点：自己写的
第三版：
直接用mapserver进行通讯
优点：简单，我基本不需要太多的学习就可以成功跑通
缺点：不好配，得写launch
对于launch文件，我选择用ai跑一个，我自己写的话需要学习python,不如由ai写再后续调整参数