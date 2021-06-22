from contextlib import nullcontext
import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import Int8
import nav2_msgs
from geometry_msgs.msg import PoseStamped

class MapCall(Node):

        def __init__(self):
                super().__init__('map_call')
                #ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 2.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
                self.busy = False
                self.goal = PoseStamped()
                self.rooms = self.readCoordinates('rooms.txt')
                self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
                self.subscriber = self.create_subscription(Int8, '/room', self.coordinate, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        def coordinate(self, msg):
                if len(self.rooms) > msg:
                        x, y = self.rooms[msg]
                        self.goal.pose.position.x = x/100.0
                        self.goal.pose.position.y = y/100.0
                        self.publisher.publish(self.goal)


        def readCoordinates(filename):
                testsite_array = []
                with open(filename) as my_file:
                        for line in my_file:
                                testsite_array.append(line)
                res = []
                for i in testsite_array:
                        i = i.replace('\n', '')
                        i = i.replace('[ ', '[')
                        i = i.replace('[ ', '[')
                        i = i.replace(']', '')
                        i = i.replace('[', '')
                        i = i.replace('  ', ' ')
                        tmpx, tmpy = i.split(' ')
                        res.append((int(tmpx), int(tmpy)))

                return res
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    coordinator = MapCall()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(coordinator)
    # Explicity destroy the node
    coordinator.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()