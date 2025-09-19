ros2 pkg create --build-type ament_cmake gluon_arm_description \
  --dependencies robot_state_publisher joint_state_publisher xacro
  
cd /home/panda/study/ros/ugv_ddsm_ws && colcon build --packages-select gluon_arm_description


ros2 pkg create --build-type ament_cmake gluon_arm_bringup

cd /home/panda/study/ros/ugv_ddsm_ws && colcon build --packages-select gluon_arm_bringup gluon_arm_description

ros2 launch gluon_arm_bringup gluon_arm.launch.py