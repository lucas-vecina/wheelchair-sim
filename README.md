# wheelchair-sim
Wheelchair Simulation repository for the subject "Projeto de Sistemas Mecatrônicos (ES965)", taught in the Automation and Control Engeenireing course at UNICAMP, Brazil.


## Description
This project contains a embeeded arduino code to read inputs from a joystick and stream data trough Serial. A ROS2 node read this data and publish it to a topic. Another control node subscribed to this topic process the data simulating a PWM converter, H-Bridge and PMDC motor and publishs a `/cmd_vel` topic with target speed and angular velocity to robot.

On Gazebo, a wheelchair model buit on top of diff_drive plugin subscribes to `/cmd_vel` topic and move accordingly to the reference, taking in consideration physical aspects, like: gravity, damping, friction, kinematics, inertia, etc.

_PlotJuggler_ is also used to subscribe to some topics and display Real-Time metrics from the simulation.

## Build and run
To deploy embeeded code, the recommendede approach is to use _PlataformIO_ IDE framework.

All the used tools are encapsulated with Docker containers. It is necessary to give _root_ user access to X server in order to enable GUIs visualizations on containerized environment.

```bash
git clone https://github.com/lucas-vecina/wheelchair-sim
cd wheelchair-sim/ros
xhost +local:root
```

To build and run ROS2 ecossystem with ROS, Gazebo and PlotJuggler, is necessary _Docker_. Build and run with _docker compose_.

```bash
docker compose up --build
```
