import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from communication_interfaces.action import CreateUserNfcTag   
import threading
from . import rest_interface 

class ros_controller(Node):

    def __init__(self):
        super().__init__('ros_controller')
        self._action_client = ActionClient(
            self, CreateUserNfcTag, 'create_user')
        
        self.reader_in_use= False
        self.reader_status = ""
        self.id = ""
        self.feedback_event = threading.Event()
        
    def call_create_nfc_tag_action(self, first_name="", last_name="", id="", id_wait_timeout=10 ):

        if(id=="" or (first_name=="" and last_name=="")):
            return ""
               
        goal_msg = CreateUserNfcTag.Goal()
       
        if(id != ""):
            goal_msg.user_id = id
            self.id= id

        if(first_name != "" and last_name != ""):
            goal_msg.first_name = first_name
            goal_msg.last_name = last_name

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.get_action_responce_callback)

        if(id==""):
            self.feedback_event.wait(id_wait_timeout)
        return self.id
        

    def get_action_responce_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if (result.task_status.is_db_user_created):
            
            self.reader_status = ""
            self.id = ""
            self.reader_in_use=False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.task_status
        self.id = feedback.user_id
        if (feedback.is_reader_completed):
            self.reader_status = "Idle"
        elif (feedback.is_reader_ready_to_write):
            self.reader_status = "Token_benötigt"
        elif (feedback.is_db_user_created):
            self.reader_status = "Setup"
        self.feedback_event.set() 

def main(args=None):
    rclpy.init()
    ros_node=ros_controller()
    web_interface = rest_interface.RestInterface(ros_node)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,))
    spin_thread.start()
    web_interface.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()