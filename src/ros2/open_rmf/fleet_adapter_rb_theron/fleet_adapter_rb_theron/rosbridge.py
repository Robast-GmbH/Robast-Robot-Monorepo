import threading
import time

from roslibpy import Header, Message, Ros, Time, Topic
from roslibpy.actionlib import ActionClient, Goal


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
        # TODO: Check if ros.close() is needed

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        return self.ros.is_connected

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        position = self.context.get("/imu/data")
        print(position)
        return self.pose

    def start_subscriber(self, topic, msg_type):
        listener = Topic(self.ros, topic, msg_type)
        listener.subscribe(lambda message: self.context.update(topic, message))

    def start_action(self, topic, action_type, goal_msg):
        action_client = ActionClient(self.ros, topic, action_type)
        goal = Goal(action_client, goal_msg)
        goal.on("feedback", lambda feedback: print("Feedback: ", feedback))
        goal.send()

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


ros_bridge = Rosbridge()
ros_bridge.start_subscriber("/imu/data", "sensor_msgs/msg/Imu")
try:
    while True:
        pass
except KeyboardInterrupt:
    ros_bridge.ros.terminate()
