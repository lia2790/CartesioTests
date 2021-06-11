
<img align="right" src="https://user-images.githubusercontent.com/15608027/118106879-62679800-b3de-11eb-845b-666ac6ad77c9.png" width="560"/>

# ➤ CartesioTestsCpp

<br />
<br />
<br />
<br />
<br />
<br />

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

**Here**, you can run **_Cartesio's C++ Minimal Example!_**

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

<br />

**Specify your own target** inside the configuration file _CartesioTestsCpp.yaml_
```c++
task_position: [0.0,0.05,0.0]
task_orientation: [0.0,0.0,90.0]
target_time: 1.0
```

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

<br />

**Start Xbot2Core** to low level control the robot

```c++
./setup_xbot2_coman.sh
```
A _setup file_ is prepared for you, just run it with the command above.

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

<br />

**To visualize on RViz** run

```c++
rosrun rviz rviz -d /absPath/CartesioTests/CartesioTestsCpp/configs/rviz/disp.rviz 

```

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

<br />

**Now control the robot!**

```c++
roslaunch cartesio_tests_cpp CartesioTestsCpp_Coman.launch
```

<img align="left" src="https://user-images.githubusercontent.com/15608027/117687394-0d4a3d00-b1b8-11eb-8691-953fc945cf71.png" width="320"/>

<br />

