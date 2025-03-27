#!/bin/bash
read -p 'Release number: ' RELEASE_NUM
RELEASE="mpac_a1_release_${RELEASE_NUM}"
cd build && make
cd ..
cp -r mpac_release_template ${RELEASE}
mkdir ${RELEASE}/robot
cp robot/robot.urdf ${RELEASE}/robot
cp -r robot/stl ${RELEASE}/robot
mkdir ${RELEASE}/mpac_core
cp -r ../mpac_core/vis ${RELEASE}/mpac_core/vis
cp analysis/live/vis.py ${RELEASE}/vis.py
cp build/ctrl ${RELEASE}/ctrl
cp build/atnmy ${RELEASE}/atnmy
cp build/tlm ${RELEASE}/tlm
cp atnmy/mpac_cmd.py ${RELEASE}/mpac_cmd.py

#git checkout gazebo
#cd build && make
#cd ..
#cp build/ctrl ${RELEASE}/ctrl_gazebo
#
#zip -r ${RELEASE}.zip ${RELEASE}
