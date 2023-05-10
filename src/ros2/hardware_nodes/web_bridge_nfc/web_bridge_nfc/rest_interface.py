from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

app = FastAPI()


class User(BaseModel):
    first_name: str
    last_name: str


class RestInterface():

    def __init__(self, ros_node):
        self.ros_node = ros_node

        @app.post("/users/", response_model=str)
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
                        status_code=400, detail="Der Inhalt der Nachricht ist nicht ist invalid")
            return {"user_id": self.ros_node.id}

        @app.post("/users/nfc/")
        def create_nfc_card_for_user(id: str):
            if (self.ros_node.reader_in_use):
                raise HTTPException(
                    status_code=503, detail="Reader in Benutzung")
            else:
                self.ros_node.call_create_nfc_tag_action(id=id)

        @app.get("/users/nfc/", response_model=str)
        def get_nfc_status(user_id: int):
            if (not self.ros_node.reader_in_use and self.ros_node.id != user_id):
                raise HTTPException(status_code=404)
            return {"Status": self.ros_node.reader_status}

    def run(self):
        uvicorn.run(app, host='0.0.0.0', port=5001, log_level='warning')
