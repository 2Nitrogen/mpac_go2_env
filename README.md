# Motion Primitives for Autonomy and Control, Unitree A1 quadruped

This repository contains the A1 quadruped specific code for the MPAC framework,
including robot description, A1 I/O, estimation, telemetry, visualization,
autonomy, and motion primitive implementations.

## Build instructions
Note: versioning is w.r.t a known working configuration, other versions of
packages may work (expect where explicitly stated).

### Build Dependencies
- cmake version >= 3.10, `apt install cmake`
- [Pinocchio: Rigid body dynamics algorthim
  library](https://github.com/stack-of-tasks/pinocchio) version >= 2.4.3,
[install via robotpkg apt
repo](https://stack-of-tasks.github.io/pinocchio/download.html)
- [osqp: quadratic program solver](https://osqp.org/docs/installation/cc++) version >= 0.6.0, build from
  source
- [osqp-eigen: eigen interface to osqp](https://github.com/robotology/osqp-eigen) version >= 0.6.1, build
  from source
- [eigen3: linear algebra library](https://eigen.tuxfamily.org) version >=
  3.3.7-2, `apt install libeigen3-dev` on Ubuntu 20.04, older Ubuntu
  version pull a package that's too old.
- [hdf5 file format storage](https://www.hdfgroup.org/solutions/hdf5/) version >= 1.10.4, `apt
  install libhdf5-dev`
- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)
- if using the gazebo environment, the core [ROS](http://wiki.ros.org/ROS/Installation)
  packages. Tested on Noetic, but should work on older versions.

### Additional Dependencies
These are required to run the supplied python visualizer and read the hdf5
telemetry files in python.
- git-lfs `apt install git-lfs`, `git lfs install`, `git lfs pull` for the stl files stored on git
- pyqtgraph, version >= 0.12.2, `pip3 install pyqtgraph`.
  - requires `pip3 install PyOpengl`
  - `pip3 install PyQt5==5.15.4`
  - `pip3 install numpy-stl`
  - `pip3 install urdfpy`
  - `pip3 install trimesh`
- h5py, version >= 2.10.0, `pip3 install h5py`
- numpy, version >= 1.17.3, `pip3 install numpy`

For gazebo sim:
- [ROS](http://wiki.ros.org/ROS/Installation)
- [Unitree ROS interface](https://github.com/unitreerobotics/unitree_ros)

### Build
1. Clone this repo
2. Clone mpac_core into the root of this repo
3. `mkdir build`
4. `cd build && cmake ..`
5. `make`

This should produce several executables (ctrl, atnmy, tlm, and any analysis
executables).

- `ctrl`: this is the realtime control process that executes the motion
  primitives and primitive switching. By default it executes in simulation
(equivalent to `--io_mode=simulation`), with the `--io_mode=hardware` flag it
will run on the physical A1.  It receives desired primitive and broadcasts
telemetry over UDP. Additional arguments to the io mode can be applied with
`--io_mode_args`, including `--io_mode_args=ground_truth` that uses a
vprn-based mocap to localize body and orientation.
- tlm: this reads the tlm broadcast from control, computes derived telemetry,
  and writes to hdf5 file. It has a `--forward_ip` argument to relay the
consumed UDP packet to another process.
- atnmy: barebones for now, but reads tlm UDP packet from control and broadcasts
  UDP packets with desired primitives to the ctrl process.
- atnmy/mpac_cmd.py: python module for sending desired primitive command
  packets. Can import into IDLE and use as a REPL or any python code
- analysis/live/vis.py: default visualizer based on pyqtgraph. Shows 3D view,
  telemetry readings, etc both online and offline. Reads directly from the hdf5
file written by tlm. An important note: the 0.11.0 version (and latest version)
have a memory leak on 64-bit linux
(https://github.com/pyqtgraph/pyqtgraph/issues/1783). Hopefully, will be fixed
and released soon. In the meantime you can patch your own version, or just don't
run the visualizer for too long :/
    
### Run Instructions
In separate terminals, running from root of mpac_a1:
1. `./build/ctrl`
2. `./build/tlm`
3. `python3 analysis/live/vis.py`. Press play and change to live stream.
4. `./build/atnmy` or start IDLE and `from atnmy.mpac_cmd import *`. Type
desired primitive on cmdline.

If you want to run the gazebo sim:
1. `roslaunch unitree_gazebo normal.launch rname:=a1`
2. `./build/ctrl --io_mode=gazebo`
3. `./build/tlm`. If you're going to rosbag for data, this isn't necessary.
4. `python3 analysis/live/vis.py`. If using ROS visualizer, this isn't
necessary.
5. `./build/atnmy` or start IDLE and `from atnmy.mpac_cmd import *`. Type
desired primitive on cmdline.

### Python Autonomy Development
You can also import `mpac_cmd.py` into any python program and call commands as
part of some autonomy program. To read raw telemetry, use the `get_tlm_data()`
function. This returns a numpy array with custom dtype -- see the mpac_cmd.py
file for details about specific elements of the array.
