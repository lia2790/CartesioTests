
<img align="right" src="https://user-images.githubusercontent.com/15608027/118106879-62679800-b3de-11eb-845b-666ac6ad77c9.png" width="580"/>

# ➤ CartesioTests
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="220"/>

<br />
<br />
<br />
<br />
<br />
<br />

**[CartesIO](https://advrhumanoids.github.io/CartesianInterface/) is a Cartesian Control Framework** with focus on **online control** of multi-chained, hyper-redundant floating-base robots. The software archicteture is mainly built on 
**ROS instruction**, providing a **robot and task description** with **[URDF](http://wiki.ros.org/urdf)**/**[SRDF](http://wiki.ros.org/srdf)** and **[YAML files](https://docs.ansible.com/ansible/latest/reference_appendices/YAMLSyntax.html)**. 
The **CartesioTests Repository** basically constitutes a **Cartesio Wrapper** of the _C++ Examples_ of the _Cartesio's Documentation_. Here, you can directly execute 
it and you can modify it just tuning some input parameters. **Here, you can find three main _sub_ packages**
- **[CartesioTestsQuickStart](https://github.com/lia2790/CartesioTests/tree/main/CartesioTestsQuickStart)**
- **[CartesioTestsROSClient](https://github.com/lia2790/CartesioTests/tree/main/CartesioTestsROSClient)**
- **[CartesioTestsCpp](https://github.com/lia2790/CartesioTests/tree/main/CartesioTestsCpp)**

which each one implements one _C++ Example_ of _Cartesio's Documentation_ plus the simple _Launch file_. _**Download the Repo to have all the examples ready to use!**_

## ➤ Table of Contents
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

* [➤ CartesioTests](#-cartesiotests)
* [➤ Table of Contents](#-table-of-contents)
* [➤ Installation](#-installation)
* [➤ Execution](#-execution)
* [➤ Dependences](#-dependences)

## ➤ Installation
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

**To Install** CartesioTests run
```c++
git clone https://github.com/lia2790/CartesioTests.git
```
inside the _**src folder**_ of a [catkin' workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and that is it.

## ➤ Execution
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

**To Execute the examples in the packages**, run

- **CartesioTestsCpp**
```c++
roslaunch cartesio_tests_cpp CartesioTestsCpp.launch
```
- **CartesioTestsQuickStart**

```c++
roslaunch cartesio_tests_quickstart CartesioTestsQuickStart.launch
```
- **CartesioTestsROSClient**
```c++
roslaunch cartesio_tests_rosclient CartesioTestsROSClient.launch
```
respectively!

## ➤ Dependences
<img aligh="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="550"/>

CartesioTests requires the following dependences:
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page): to handle with basic algebra;
* [CartesIO](https://advrhumanoids.github.io/CartesianInterface/): to handle robot tasks;
* [XBOT2](https://github.com/ADVRHumanoids/xbot2_wip): to handle robot low level control;

