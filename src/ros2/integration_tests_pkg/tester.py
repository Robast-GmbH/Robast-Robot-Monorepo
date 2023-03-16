#! /usr/bin/env python3

import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class WaypointFollowerTest(Node):

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_tester', namespace='')
        self.waypoints = None
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.goal_handle = None
        self.action_result = None
        self.param_cli = self.create_client(SetParameters,
                                            '/waypoint_follower/set_parameters')
        time.sleep(5)
        
    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self, block, cancel):
        # if not self.waypoints:
        #     rclpy.error_msg('Did not set valid waypoints before running test!')
        #     return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'follow_waypoints' action server not available, waiting...")

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()
        if cancel:
            time.sleep(2)
            self.cancel_goal()

        self.info_msg("Waiting for 'follow_waypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
            self.action_result = result
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg(f'Goal failed with status code: {status}')
            return False
        if len(self.action_result.missed_waypoints) > 0:
            self.info_msg('Goal failed to process all waypoints,'
                          ' missed {0} wps.'.format(len(self.action_result.missed_waypoints)))
            return False

        self.info_msg('Goal succeeded!')
        return True


    def setStopFailureParam(self, value):
        req = SetParameters.Request()
        req.parameters = [Parameter('stop_on_failure',
                                    Parameter.Type.BOOL, value).to_parameter_msg()]
        future = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def shutdown(self):
        self.info_msg('Shutting down')

        self.action_client.destroy()
        self.info_msg('Destroyed follow_waypoints action client')

        self.shutdown_lifecycle_nodes("lifecycle_manager_navigation")
        # self.shutdown_lifecycle_nodes("lifecycle_manager_localization")
        # self.shutdown_lifecycle_nodes("lifecycle_manager_slam_toolbox")

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)

    def shutdown_lifecycle_nodes(self, name: str):
        transition_service = name +'/manage_nodes'
        self.info_msg(f'starting shutdown of {transition_service}')
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'{transition_service} service call failed {e!r}')

        self.info_msg(f'{transition_service} finished')

def main(argv=sys.argv[1:]):
    rclpy.init()
    # should I track the time for each scenario?
    # wait a few seconds to make sure entire stacks are up
    time.sleep(15)

    wps = [[-0.52, -0.54], [7.5, -1.5], [0.58, 0.52]]
    starting_pose = [0.0, 0.0]

    test = WaypointFollowerTest()
    test.setWaypoints(wps)
    
    result = False

    # preempt with new point
    test.setWaypoints([starting_pose])
    result = test.run(True, False)
    time.sleep(2)
    
    test.info_msg('Starting navigate through tight space test')
    test.setWaypoints([wps[1]])
    result = test.run(True, False)

    if not result:
        test.info_msg('Couldnt reach the room in front of robast')
        
    test.info_msg('Finished navigate through tight space test')

    # Zero goal test
    test.info_msg('Starting zero goal test')
    test.setWaypoints([])
    result = test.run(True, False)
    test.info_msg('Finished zero goal test')

    # Cancel test
    test.info_msg('Starting cancel Test')
    test.setWaypoints(wps)
    result = test.run(True, True)
    assert not result
    result = not result
    test.info_msg('Finished cancel Test')
    
    test.shutdown()
    test.info_msg('Done Shutting Down.')

    if not result:
        test.info_msg('Exiting failed')
        exit(1)
    else:
        test.info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()