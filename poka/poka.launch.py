# -----------------------------------------------
# ROS LAUNCH
# for POKA
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------
# https://ubuntu.com/blog/ros2-launch-required-nodes
# https://gbiggs.github.io/rosjp_ros2_intro/ros2_tools.html#launch
# -----------------------------------------------

"""Launch a add_two_ints_server and a (synchronous) add_two_ints_client."""

import launch
import launch_ros.actions


def generate_launch_description():

    gpioout = launch_ros.actions.Node(
        package='poka', node_executable='gpioout', output='screen')
    gpioin = launch_ros.actions.Node(
        package='poka', node_executable='gpioin', output='screen')
    gpioi2c = launch_ros.actions.Node(
        package='poka', node_executable='gpioi2c', output='screen')
    #gpiotest = launch_ros.actions.Node(
    #    package='poka', node_executable='gpiotest', output='screen')
    procconsole = launch_ros.actions.Node(
        package='poka', node_executable='procconsole', output='screen')
    procpoka = launch_ros.actions.Node(
        package='poka', node_executable='procpoka', output='screen')

    return launch.LaunchDescription([
        gpioout,
        gpioin,
        gpioi2c,
        #gpiotest,
        procconsole,
        procpoka,
        # TODO(wjwwood): replace this with a `required=True|False` option on ExecuteProcess().
        # Shutdown launch when client exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=procpoka,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])
