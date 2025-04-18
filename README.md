## Description
This repository contains a 9-DOF inverted robotic arm-gantry simulator for visulizing motion profiles using ROS kinetic, Gazebo, and moveit. This simulator was developed as part of the System for Arbitrary Motion Profiles in Littoral Environments (SAMPLE) project at Oklahoma State University.
Contributors: Shahbaz P Qadri Syed, He Bai
Last Updated: 2022

## Demo
9-DOF Robotic arm-gantry system:

![agmodel](https://github.com/user-attachments/assets/02b4113a-0378-4aa1-89f1-52a7f3ed9fb4)

Circular trajectory tracking:


https://github.com/user-attachments/assets/ea48dda2-cc6c-4e1d-9811-5feb54c139ca




Sinusoidal trajectory tracking:


https://github.com/user-attachments/assets/7bff6f86-4158-422a-89b9-ecf21c4b4684



## Quickstart
```
cd robotcontrolpkg/
catkin build
```

Open new terminal and enter the following commands to launch the gazebo simulation
```source devel/setup.bash
roslaunch robot_moveit demo_gazebo.launch
```
To command custom trajectory on the simulator, enter the following commands in another terminal
```
source devel/setup.bash
cd robotcontrolpkg/src/scripts/src
chmod +x custom_trajectory.py
python custom_trajectory.py
```

To use custom algorithms e.g., Approximate inference for control (AICO) algorithm [Toussaint, 2009], modify robotcontrolpkg with inferencerobotcontrolpkg and run the following
```
source devel/setup.bash
cd inferencerobotcontrolpkg/src/scripts/src
chmod +x AICO_Roboticarm_gantry.py
python AICO_Roboticarm_gantry.py
```

If you are using this simulator in your work, please cite:
```
@mastersthesis{syed2022development,
  title={Development of a robotic arm-gantry simulator with probabilistic inference based control},
  author={Syed, Shahbaz Peeran Qadri},
  year={2022},
  school={Oklahoma State University}
}
```

