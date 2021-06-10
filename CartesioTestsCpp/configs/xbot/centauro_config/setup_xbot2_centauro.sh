#!/bin/sh
set_xbot2_config /home/liana/catkin_ws/src/CartesioTests/CartesioTestsCpp/configs/xbot/centauro_config/centauro_basic.yaml
export  XBOT_ROOT=/home/liana/catkin_ws/src/CartesioTests/CartesioTestsCpp
valgrind xbot2-core --hw dummy