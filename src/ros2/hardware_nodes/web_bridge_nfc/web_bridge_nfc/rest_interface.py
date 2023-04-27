import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from communication_interfaces.action import CreateUserNfcTag

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

import threading


app = FastAPI()


class User(BaseModel):
    id: str = None
    first_name: str
    last_name: str


class RestInterface(Node):

    def __init__(self):
        super().__init__('rest_interface')
        self._action_client = ActionClient(
            self, CreateUserNfcTag, 'create_user')
        self.reader_status = ""

        @app.post("/users/")
        async def create_user(user: User):
            if (self.reader_status != ""):
                raise HTTPException(
                    status_code=503, detail="Reader in Benutzung")
            else:
                self.make_nfc( first_name= user.first_name, last_name= user.last_name)

        @app.post("/users/nfc/")
        async def create_nfc_card_for_user(id: str):
            if (self.reader_status != ""):
                raise HTTPException(
                    status_code=503, detail="Reader in Benutzung")
            else:
                 self.make_nfc( id= id)


        @app.get("/users/nfc/", response_model=str)
        async def get_nfc_status(user_id: int):
            if (self.reader_status == ""):
                raise HTTPException(status_code=404)
            return self.reader_status
            

    def make_nfc( self, first_name="", last_name="", id=""):
        self.get_logger().info(id)
        self.get_logger().info(first_name)
        goal_msg = CreateUserNfcTag.Goal()
        if(id!=""):
            goal_msg.user_id = id

        if(first_name!="" and last_name!=""):
            goal_msg.first_name = first_name
            goal_msg.last_name = last_name

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.get_action_responce_callback)
        

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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.task_status
        if (feedback.is_reader_completed):
            self.reader_status = "Idle"
        elif (feedback.is_reader_ready_to_write):
            self.reader_status = "Token_benötigt"
        elif (feedback.is_db_user_created):
            self.reader_status = "Setup"


def main(args=None):
    rclpy.init()
    rest_interface = RestInterface()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(rest_interface,))
    spin_thread.start()
    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
