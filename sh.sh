#!/bin/bash

# 在同一个终端中打开三个标签页，并分别执行指令

# 打开第一个标签页并执行第一个指令
gnome-terminal --tab --title="rviz" -- bash -c "source devel/setup.bash; chmod +x sh_utils/pub_triger.sh; roslaunch mapping rviz_sim.launch; exec bash"

# 等待3秒
sleep 3

# 打开第二个标签页并执行第三个指令
gnome-terminal --tab --title="fake_target" -- bash -c "source devel/setup.bash; roslaunch planning fake_car_target.launch; exec bash"
sleep 3

# 打开第三个标签页并执行第二个指令
gnome-terminal --tab --title="simulation1" -- bash -c "source devel/setup.bash; roslaunch planning simulation1.launch; exec bash"

# 等待1秒
sleep 3

# 打开第四个标签页
gnome-terminal --tab --title="pub_triger" -- bash -c "./sh_utils/pub_triger.sh; exec bash"
