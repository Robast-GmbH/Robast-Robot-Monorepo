import threading
import time

from roslibpy import Header, Message, Ros, Time, Topic
from roslibpy.actionlib import ActionClient, Goal
from geometry_msgs.msg import Pose


class ThreadSafeDict:
    def __init__(self):
        self.lock = threading.Lock()
        self.dict = {}

    def update(self, key, value):
        with self.lock:
            self.dict[key] = value

    def get(self, key):
        with self.lock:
            return self.dict.get(key)


class Rosbridge:
    def __init__(self):
        # create ROS connection
        # self.context = dict(wait=threading.Event(), counter=0)
        self.context = ThreadSafeDict()
        self.ros = Ros("127.0.0.1", 9090)
        self.ros.run()
        self.ros.on_ready(lambda: print("Is ROS connected?", self.ros.is_connected))
        self.start_subscriber("/robot_position", "geometry_msgs/msg/Point")
        self.start_subscriber(
            "/navigation_remaining_time", "builtin_interfaces/msg/Duration"
        )
        self.start_subscriber("/is_navigating", "std_msgs/msg/Bool")
        self._goal_pose_publisher = Topic(
            self.ros, "/set_goal_pose", "geometry_msgs/msg/Pose"
        )
        # TODO: Check if ros.close() is needed

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        return self.ros.is_connected

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        position = self.context.get("/robot_position")
        offset = [23.3, -7.33]
        if position is None:

            return [offset[0], offset[1], 0.0]
        # position = [
        #     offset[0] + position["x"] * 20.0,
        #     -offset[1] + position["y"] * 20.0,
        #     position["z"],
        # ]
        # return position
        # print(position)
        return [position["x"], position["y"], position["z"]]

    def start_subscriber(self, topic, msg_type):
        listener = Topic(self.ros, topic, msg_type)
        listener.subscribe(lambda message: self.context.update(topic, message))

    def navigate_to_goal_pose(self, robot_name, goal_pose):
        self.context.update("/is_navigating", {"data": True})
        goal_msg = Message(
            {
                "position": {
                    "x": (goal_pose[0] - 466.0) * 0.05,
                    "y": (goal_pose[1] + 179.4) * 0.05,
                    "z": 0.0,
                },
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        )
        self._goal_pose_publisher.publish(goal_msg)

    def is_navigating(self):
        return self.context.get("/is_navigating")["data"]

    def start_action(self, topic, action_type, goal_msg, feedback_cb):
        action_client = ActionClient(self.ros, topic, action_type)
        goal = Goal(action_client, goal_msg)
        goal.on("feedback", feedback_cb)
        goal.send()

    def get_remaining_nav_time(self):
        remaining_time = self.context.get("navigation_remaining_time")
        if remaining_time is None:
            return 0
        else:
            return remaining_time["sec"]

    # def retrieve_data(self,topic,msg_type):
    #     context = dict(wait=threading.Event(), counter=0)
    #     listener = Topic(self.ros, topic, msg_type)

    #     def receive_message(message):
    #         context["value"] = message
    #         listener.unsubscribe()
    #         context["wait"].set()

    #     def start_receiving():
    #         listener.subscribe(receive_message)

    #     t1 = threading.Thread(target=start_receiving)
    #     t1.start()

    #     if not context["wait"].wait(10):
    #         raise Exception

    #     t1.join()
    #     return context["value"]

    def publish_data(self, topic, msg_type, data):
        publisher = Topic(self.ros, topic, msg_type)
        message = Message(data)
        publisher.publish(message)


# ros_bridge = Rosbridge()
# ros_bridge.start_subscriber("/robot_position", "geometry_msgs/msg/Point")
# try:
#     while True:
#         time.sleep(1)
#         print(ros_bridge.context.get("/robot_position"))

# except KeyboardInterrupt:
#     ros_bridge.ros.terminate()
