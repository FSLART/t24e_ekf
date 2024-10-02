# T24-e State Estimator (Extended Kalman Filter)

This the the implementation of the state estimator for the T-24e car. It uses an Extended Kalman Filter. All the implementation was done from scratch.

The implementation is in C++ and the matrix operations are handled by the well-known Eigen linear algebra library.

A Kalman Filter needs two models: the **motion model** and the **measurement model**. From this point on the terminology car, vehicle or system denote the same thing.

In this case, the state of the vehicle is characterized by its position in 2D, yaw angle and speed.

The **motion model** predicts the motion of the system using the knowledge of the system dynamics. Considering the system is in this case a car with ackermann steering, the simplest solution was chosen, which is the kinematic bicycle model. 
The motion model relies on the motor RPM measurements for vehicle speed calculation and uses the steering actuator encoder measurements for steering angle calculation.

The **measurement model** maps the state of the vehicle into the measurement variables. That is, it creates expected measurements. The measurements correct the filter predictions. For this, the GNSS/INS was used.

## Interfacing
### Subscribers
- /dynamics (lart_msgs/DynamicsCMD)
- /gnss_ins (lart_msgs/GNSSINS)

### Publishers
- /state_estimate (geometry_msgs/PoseWithCovarianceStamped)

### Transforms
- map -> base_link

## Building and Running

### Bare metal
#### Requisites
- ROS Humble
- Eigen 3
- [lart_common](https://github.com/FSLART/lart_common) (comes bundled as a submodule, no need to worry)
- [lart_msgs](https://github.com/FSLART/lart_msgs)

#### Building
- Navigate to the workspace directory.
- Run the command ```colcon build --symlink-install --parallel-workers 6```.

#### Running
- Run the command ```source install/setup.bash```.
- Run the command ```ros2 run t24e_ekf state_estimator```.

### Docker
#### Requisites
- Docker

#### Building
- Run the command ```docker build -t t24e_ekf .```.

#### Running
- Run the command ```docker run --rm -it --network=host --pid=host --ipc=host t24e_ekf```.
- Make sure you are in the root of the workspace inside the container.
- Inside the container, run the commands:
    - ```source /opt/ros/humble/setup.bash```
    - ```ros2 run t24e_ekf state_estimator```

