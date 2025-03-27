# Running from release

Note: Ubuntu 20.04 is required, otherwise libraries are incompatible. Newer versions possibly work.

`export LD_LIBRARY_PATH=<path_to_release_lib>:$LD_LIBRARY_PATH`

You must do this in each terminal or add to ~/.bashrc. This step isn't necessary
if you have the libraries installed on your system.

You will still need the python dependencies if you want to use the python
visualizer.

Likewise, you will need the ROS dependencies for the ctrl_gazebo.

Otherwise, run with the same run instructions as in the mpac_a1 readme, except
binaries are in the root directory instead of build (so ./ctrl instead of
./build/ctrl).
