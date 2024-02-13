# ITMO ros course 3rd semester labs variant 1
lab 1: node_interaction  
lab 2: turtle_controller  
lab 3: robot_model  
lab 4: robot_sim_control  
project: robot_sim_track  
Project contributors:
- Alexey Rumyantsev (code)
- Dmitrii Chebanenko (code)
- Pavel Ovchinnikov (debug)  

Clone everything into 'catkin_ws/src' and write "catkin build" and "source ~/catkin_ws/devel/setup.bash"  

In WSL with Xming configuration write "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" to see rviz and gazebo
