A pointcloud pipeline demo.

Includes bringup scripts for a Gazebo simulation and [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) along with a mock image filter node that sits between images and the stereo pipeline.

## Depends

- ROS 2 (Foxy or later)

## Build

Source ROS 2 installation, for example:

        source foxy_ws/install/setup.bash

Fetch source dependencies:

        mkdir -p pc_pipe_ws/src
        cd pc_pipe_ws
        wget https://raw.githubusercontent.com/jacobperron/pc_pipe/master/pc_pipe.repos
        vcs import src < pc_pipe.repos

Build:

        colcon build

## Launch simulation

Do this once per shell:

        source pc_pipe_ws/install/setup.bash

Launch the pipeline, including Gazebo simulator:

        ros2 launch pc_pipe bringup.launch.xml start_gazebo:=true viz:=true

Optional start a subscription to confirm point cloud messages are received:

        ros2 ros2 topic echo --qos-reliability reliable --no-arr /mycamera/points2

Trigger a stereo image capture:

        ros2 topic pub -1 mycamera/image_trigger std_msgs/msg/Empty
