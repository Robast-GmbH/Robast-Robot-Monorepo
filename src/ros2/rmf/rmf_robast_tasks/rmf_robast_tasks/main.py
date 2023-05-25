import sys
from rclpy.parameter import Parameter
import argparse
from task_requester import TaskRequester
from task_builder import fill_compose, fill_go_to_place
import rclpy

def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--start_coord', required=False,
                            type=str, nargs='+',
                            help="Start coordidates")
    parser.add_argument('-e', '--end_coord', required=False,
                            type=str, nargs='+',
                            help="end coordinates")
    parser.add_argument('-F', '--fleet', type=str,
                            help='Fleet name, should define tgt with robot')
    parser.add_argument('-R', '--robot', type=str,
                            help='Robot name, should define tgt with fleet')
    parser.add_argument('-st', '--start_time',
                            help='Start time from now in secs, default: 0',
                            type=int, default=0)
    parser.add_argument('-pt', '--priority',
                            help='Priority value for this request',
                            type=int, default=0)

    args = parser.parse_args(args_without_ros[1:])
    # check user delivery arg inputs
    if (len(args.start_coord)>=7 ):
        task_requester.get_logger().error("Invalid start_coord")
        parser.print_help()
        sys.exit(1)
    if (len(args.end_coord)>=7 ):
        task_requester.get_logger().error("Invalid end_coord")
        parser.print_help()
        sys.exit(1)

    sub_tasks = []
    sub_tasks.append(fill_go_to_place(args))

    task= fill_compose(sub_tasks,"let the robot go to a place and let it perform user action")
    task_requester = TaskRequester( task, "compose", args)
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=5.0)
    if task_requester.response.done():
        print(f'Got response:\n{task_requester.response.result()}')
    else:
        print('Did not get a response')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)