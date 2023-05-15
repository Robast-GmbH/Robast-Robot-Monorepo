from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn


class User(BaseModel):
    first_name: str
    last_name: str


class User_id(BaseModel):
    id: str


class RestInterface():

    def __init__(self, ros_node, app=FastAPI()):
        self.ros_node = ros_node
        self.app = app

        @self.app.post("/users/")
        def create_user(user: User):
            if (self.ros_node.reader_in_use):
                raise HTTPException(
                    status_code=503, detail="Reader in Benutzung")
            else:
                self.ros_node.reader_in_use = True
                self.ros_node.call_create_nfc_tag_action(
                    first_name=user.first_name,
                    last_name=user.last_name)

                if(self.ros_node.id == ""):
                    raise HTTPException(
                        status_code=400, detail="Ungueltige angaben")
            return {"user_id": self.ros_node.id}

        @self.app.post("/users/nfc/")
        def create_nfc_card_for_user(id: User_id):
            if (self.ros_node.reader_in_use):
                raise HTTPException(
                    status_code=503, detail="Reader in Benutzung")
            elif(len(id.id) < 1):
                raise HTTPException(status_code=404)
            else:
                self.ros_node.call_create_nfc_tag_action(id=id.id)
                if(self.ros_node.id == ""):
                    raise HTTPException(
                        status_code=400, detail="Die ID ist nicht vergeben")

        @self.app.get("/users/nfc/{user_id}")
        def get_nfc_status(user_id: int):
            if (not self.ros_node.reader_in_use or self.ros_node.id != user_id):
                raise HTTPException(status_code=404)
            return {"Status": self.ros_node.reader_status}

    def run(self, host='0.0.0.0', port=5001, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app
