#!/bin/sh
set_xbot2_config /home/liana/catkin_ws/src/CartesioTests/CartesioTestsCpp/configs/xbot/teleop2_config/teleop2_basic.yaml
export  XBOT_ROOT=/home/liana/catkin_ws/src/CartesioTests/CartesioTestsCpp
xbot2-core --verbose --hw dummy