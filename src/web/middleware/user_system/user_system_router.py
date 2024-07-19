from fastapi import APIRouter, HTTPException

from user_system.user_repository import UserRepository
from pydantic_models.user_request_models import (
    CreateUserRequest,
    DeleteUserRequest,
    UpdateUserRequest,
)
from pydantic_models.user import User
from user_system.auth_session_manager import AuthSessionManager

user_system_router = APIRouter()
user_repository = UserRepository()
auth_session_manager = AuthSessionManager()


@user_system_router.get("/all_users", tags=["Users"], response_model=list[User])
def get_all_users():
    return user_repository.get_all_users()


@user_system_router.get("/user", tags=["Users"], response_model=User)
def get_user(user_id: str):
    user = user_repository.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@user_system_router.put("/update_user", tags=["Users"], response_model=User)
def update_user(request: UpdateUserRequest):
    user = user_repository.update_user(
        user_id=request.id,
        title=request.title,
        first_name=request.first_name,
        last_name=request.last_name,
        station=request.station,
        room=request.room,
        user_groups=request.user_groups,
    )
    if not user:
        raise HTTPException(
            status_code=404, detail="User not found or could not be updated"
        )
    return user


@user_system_router.post("/create_user", tags=["Users"], response_model=User)
def create_user(request: CreateUserRequest):
    user = user_repository.create_user(
        title=request.title,
        first_name=request.first_name,
        last_name=request.last_name,
        station=request.station,
        room=request.room,
        user_groups=request.user_groups,
    )
    if not user:
        raise HTTPException(status_code=400, detail="User could not be created")
    return user


@user_system_router.post("/delete_user", tags=["Users"])
def delete_user(request: DeleteUserRequest):
    success = user_repository.delete_user(user_id=request.id)
    if not success:
        raise HTTPException(
            status_code=404, detail="User not found or could not be deleted"
        )
    return {"message": "User deleted successfully", "status": "success"}


@user_system_router.get("/session", tags=["Auth"])
def get_session(robot_name: str):
    session = auth_session_manager.get_session(robot_name)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
    return session


@user_system_router.post("/end_session", tags=["Auth"])
def post_end_session(robot_name: str):
    auth_session_manager.end_session(robot_name)
    return {"message": "Session ended successfully", "status": "success"}
