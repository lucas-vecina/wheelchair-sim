FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y \
    ros-humble-twist-mux \
    python3-venv

COPY arduino_driver/ /ros_ws/src/arduino_driver/

WORKDIR /ros_ws/src/arduino_driver/

RUN python3 -m venv .venv && \
    touch .venv/COLCON_IGNORE && \
    .venv/bin/python3 -m pip install -r requirements.txt

WORKDIR /ros_ws 

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

CMD ["/bin/bash"]
