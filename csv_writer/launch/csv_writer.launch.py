import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition  # Added this import for conditions
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition, State

def generate_launch_description(): 
    # Declare filepath to parameters
    params_path = os.path.join(get_package_share_directory(
      'csv_writer'), 'config', 'CsvWriter_param.yaml')

    # Launch Node
    csv_writer_node = LifecycleNode(
        package = "csv_writer", 
        executable = "CsvWriter", 
        name = "CsvWriter", 
        namespace="",
        output = "screen", 
        parameters = [params_path]
    )

    # Register event handler to automatically configure the node when it starts
    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=csv_writer_node,
            on_start=[
                LogInfo(msg="Configuring CsvWriterLifecycle node..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'CsvWriter',
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )

    CsvWriter_trans_event_configure = RegisterEventHandler(OnProcessStart(
        target_action=csv_writer_node, 
        on_start=[
            LogInfo(msg="Configuring CsvWriterLifecycle node..."),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(csv_writer_node),
                    transition_id=Transition.TRANSITION_CONFIGURE
            )
        )]
    ))

    # Register event handler to automatically activate the node after it's configured
    CsvWriter_trans_event_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=csv_writer_node, 
            start_state="configuring", goal_state="inactive", 
            entities=[
                LogInfo(msg="Node configured succesfully, activating..."), 
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(csv_writer_node), 
                        transition_id=Transition.TRANSITION_ACTIVATE
                    ))
            ]
        )
    )

    # Register event handler for graceful shutdown (deactivate and cleanup)
    # deactivate_event_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=csv_writer_node,
    #         on_exit=[
    #             LogInfo(msg="Shutting down CsvWriterLifecycle node..."),
    #             EmitEvent(
    #                 event=ChangeState(
    #                     lifecycle_node_matcher=launch.events.matches_action(csv_writer_node),
    #                     transition_id=Transition.TRANSITION_DEACTIVATE
    #                 )
    #             )
    #         ]
    #     )
    # )

    # Create the launch description
    ld = LaunchDescription()
    
    # Add node and event handlers
    ld.add_action(csv_writer_node)
    ld.add_action(CsvWriter_trans_event_configure)
    ld.add_action(CsvWriter_trans_event_activate)
    # ld.add_action(deactivate_event_handler)

    return ld