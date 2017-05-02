#!/bin/bash
catkin_make_isolated --install --install-space /opt/ros/ev3infoumons \
  -DCMAKE_BUILD_TYPE=Release
