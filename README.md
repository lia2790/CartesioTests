# ➤ CartesioTests
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

**Cartesio is a Cartesian Control Framework** with focus on **online control** of multi-chained, hyper-redundant floating-base robots. The main features regards 
the possibility of handling **multi tasks execution** enabling **different levels of priorities** while satisfying constraints. The software archicteture is build on 
**ROS instruction**, providing a **robot and task description** with **URDF** and **SRDF files**, the software accounts for an autonomous execution of the robot control 
for achieving the desired goal. The **CartesioTests Repository** basically constitutes a **Cartesio Wrapper**. Specifing the robot and the robot target, the CartesioTests
executes all CartesIO routines for **achieving the desired target hiding them to the user** which should **only specify** the **required inputs** and **execute ROS node**.

## ➤ Table of Contents
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

* [➤ CartesioTests](#-cartesiotests)
* [➤ Table of Contents](#-table-of-contents)
* [➤ Installation](#-installation)
* [➤ Dependences](#-dependences)
* [➤ Execution](#-execution)

## ➤ Installation
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

**To Install** CartesioTests run
```c++
git clone https://github.com/lia2790/CartesioTests.git
```
inside the _**src folder**_ of a [catkin'](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) workspace and that is it.

## ➤ Dependences
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

CartesioTests requires the following dependences:
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page): to handle with basic algebra;
* [CartesIO](https://advrhumanoids.github.io/CartesianInterface/): to handle robot tasks;


## ➤ Execution
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

**To Execute** CartesioTests run 

```c++
roslaunch cartesio_tests CartesioTests.launch
```
