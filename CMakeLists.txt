cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(a1 VERSION 1.0)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

include_directories(mpac_core/robot)
include_directories(mpac_core/sim)
include_directories(mpac_core/ctrl)
include_directories(mpac_core/atnmy)
include_directories(mpac_core/dynamics)
include_directories(mpac_core/kinematics)
include_directories(mpac_core/robot)
include_directories(mpac_core/ipc)
include_directories(mpac_core/math_utils)
include_directories(mpac_core/io)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM /opt/openrobots/include)
include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include/osqp)
include_directories(/home/hjang4/osqp/include/private)
include_directories(/usr/local/include/OsqpEigen)

include_directories(${HDF5_INCLUDE_DIRS})
include_directories(${QsqpEigen_INCLUDE_DIRS})

# includes for ROS
# set(ROS_INCLUDE_DIR /opt/ros/noetic/include)
# set(ROS_LIB_DIR /opt/ros/noetic/lib)
# include_directories(/opt/ros/noetic/include)
# include_directories(ROS_LIB_DIR)

# find_library(ROSCPP roscpp ROS_LIB_DIR)
# find_library(ROSTIME rostime ROS_LIB_DIR)
# find_library(ROSCONSOLE rosconsole ROS_LIB_DIR)
# find_library(ROSSERIAL roscpp_serialization ROS_LIB_DIR)

# set(ROS_LIBS ${ROSCPP} ${ROSTIME} ${ROSCONSOLE} ${ROSSERIAL})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROBOT='\"a1\"'")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM -DBOOST_MPL_LIMIT_LIST_SIZE=30")



file(GLOB_RECURSE OBJ_SRC
    "mpac_core/sim/*.cpp"
    "mpac_core/robot/*.cpp"
    "mpac_core/dynamics/*.cpp"
    "mpac_core/kinematics/*.cpp"
    "mpac_core/ipc/*.cpp"
    "mpac_core/math_utils/*.cpp"
    "ctrl/*.cpp"
    "robot/*.cpp"
    "tlm/*.cpp"
    "estim/*.cpp"
)

file(GLOB CTRL_SRC
    "mpac_core/ctrl/*.cpp"
    # "io/*.cpp"
    "io/io_handler.cpp"
    "io/sim_io.cpp"
    # "io/hardware_io.cpp"
    "io/mujoco_io.cpp"
)

add_library(objects OBJECT ${OBJ_SRC})
target_include_directories(objects
  PRIVATE
    ${Python3_INCLUDE_DIRS}
)
target_link_libraries(objects
  PRIVATE
    pybind11::embed
    Python3::Python
)


include_directories(io/include)
set(UNITREE_LIBS -pthread lcm)

add_executable(ctrl ${CTRL_SRC})
# find_library(UNITREE_LIB libunitree_legged_sdk_amd64.so io/lib)
find_library(PINOCCHIO_LIB libpinocchio_default.so /opt/openrobots/lib/)
target_link_libraries(ctrl objects urdfdom_model ${PINOCCHIO_LIB} m boost_system ${HDF5_LIBRARIES} OsqpEigen::OsqpEigen ${CMAKE_DL_LIBS} ${ROS_LIBS} unitree_sdk2 pybind11::embed Python3::Python)

file(GLOB ATNMY_SRC
    "mpac_core/atnmy/*.cpp"
    "mpac_core/ctrl/ctrl_modes_core.cpp"
    "mpac_core/ctrl/ctrl_mode_graph_core.cpp"
    "mpac_core/ctrl/*/*_args.cpp"
    "io/sim_io.cpp"
    "atnmy/*.cpp"
)

add_executable(atnmy ${ATNMY_SRC})
target_link_libraries(atnmy objects urdfdom_model ${PINOCCHIO_LIB} m boost_system ${UNITREE_LIBS} ${HDF5_LIBRARIES} OsqpEigen::OsqpEigen ${CMAKE_DL_LIBS} ${ROS_LIBS}) #TODO fix this, probably need robot_types.h

file(GLOB TLM_SRC
    "mpac_core/tlm/*.cpp"
    "mpac_core/ctrl/ctrl_modes_core.cpp"
    "mpac_core/ctrl/ctrl_mode_graph_core.cpp"
    "io/sim_io.cpp"
)

add_executable(tlm ${TLM_SRC})
target_link_libraries(tlm objects urdfdom_model ${PINOCCHIO_LIB} m boost_system ${UNITREE_LIBS} ${HDF5_LIBRARIES} OsqpEigen::OsqpEigen ${CMAKE_DL_LIBS} ${ROS_LIBS}) #TODO fix this, probably need robot_types.h


file(GLOB UNIT_TEST_SRC
    "analysis/unit_test/unit_test.cpp"
    "mpac_core/ctrl/ctrl_modes_core.cpp"
    "mpac_core/ctrl/ctrl_mode_graph_core.cpp"
    "mpac_core/ctrl/*/*_args.cpp"
    "io/sim_io.cpp"
)

add_executable(unit_test ${UNIT_TEST_SRC})
target_link_libraries(unit_test objects urdfdom_model ${PINOCCHIO_LIB} m boost_system ${UNITREE_LIBS} ${HDF5_LIBRARIES} OsqpEigen::OsqpEigen ${CMAKE_DL_LIBS} ${ROS_LIBS}) #TODO fix this, probably need robot_types.h
