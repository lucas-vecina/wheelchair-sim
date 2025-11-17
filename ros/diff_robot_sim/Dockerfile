FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y \
    ros-humble-joint-state-publisher-gui \
    ros-humble-twist-mux \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros-pkgs

WORKDIR /ros_ws

COPY diff_robot_sim/ src/diff_robot_sim/

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

CMD ["/bin/bash", "-c", "source /ros_ws/install/setup.sh && ros2 launch diff_robot_sim launch_sim.launch.py"]