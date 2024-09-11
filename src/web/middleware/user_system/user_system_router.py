import uuid
from fastapi import APIRouter, HTTPException
import requests

from user_system.user_repository import UserRepository
from pydantic_models.user_request_models import (
    CreateUserRequest,
    DeleteUserRequest,
    LoginRequest,
    UpdateUserRequest,
)
from pydantic_models.user_response import UserResponse, AuthResponse
from pydantic_models.default_response import DefaultResponse
from user_system.auth_session_manager import AuthSessionManager
from models.url_helper import URLHelper

user_system_router = APIRouter()
user_repository = UserRepository()
auth_session_manager = AuthSessionManager()


@user_system_router.get("/all_users", tags=["Users"], response_model=list[UserResponse])
def get_all_users():
    return user_repository.get_all_users()


@user_system_router.get(
    "/user",
    tags=["Users"],
    response_model=UserResponse,
)
def get_user(user_id: str):
    user = user_repository.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@user_system_router.put("/update_user", tags=["Users"], response_model=UserResponse)
def update_user(request: UpdateUserRequest):
    user = user_repository.update_user(
        user_id=request.id,
        nfc_id=request.nfc_id,
        mail=request.mail,
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


@user_system_router.post("/create_user", tags=["Users"], response_model=UserResponse)
def create_user(request: CreateUserRequest):
    user = user_repository.create_user(
        nfc_id=request.nfc_id,
        mail=request.mail,
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


@user_system_router.post("/delete_user", tags=["Users"], response_model=DefaultResponse)
def delete_user(request: DeleteUserRequest):
    success = user_repository.delete_user(user_id=request.id)
    if not success:
        raise HTTPException(
            status_code=404, detail="User not found or could not be deleted"
        )
    return DefaultResponse(message="User deleted successfully", status="success")


@user_system_router.post(
    "/read_and_assign_user_nfc_id", tags=["Users"], response_model=DefaultResponse
)
def read_and_assign_user_nfc_id(robot_name: str, user_id: str):
    user = user_repository.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    else:
        robot_url = URLHelper.get_robot_url(robot_name)
        response = requests.get(f"{robot_url}/read_nfc_tag?timeout_in_s=30").json()
        message = "NFC assignment failed"
        if response["status"] == "success":
            message = "NFC assignment successful"
            user_repository.update_user(
                user_id=user_id, nfc_id=response["nfc_tag"]["data"]
            )
        return DefaultResponse(message=message, status=response["status"])


@user_system_router.get("/session", tags=["Auth"], response_model=AuthResponse)
def get_session(robot_name: str):
    session_user = auth_session_manager.get_session(robot_name)
    return AuthResponse(
        status="success" if session_user else "failure", user=session_user
    )


@user_system_router.post(
    "/try_start_session", tags=["Auth"], response_model=DefaultResponse
)
def post_try_start_session(
    robot_name: str, required_user_ids: list[str], required_user_groups: list[str]
):
    result = auth_session_manager.try_start_session(
        robot_name, required_user_ids, required_user_groups
    )
    return DefaultResponse(message=result["message"], status=result["status"])


@user_system_router.post("/end_session", tags=["Auth"], response_model=DefaultResponse)
def post_end_session(robot_name: str):
    auth_session_manager.end_session(robot_name)
    return DefaultResponse(message="Session ended successfully", status="success")


@user_system_router.post("/login", tags=["Auth"], response_model=AuthResponse)
def post_login(request: LoginRequest):
    user = user_repository.login_user(request.mail, request.password)
    return AuthResponse(status="success" if user else "failure", user=user)


@user_system_router.post(
    "/change_password", tags=["Auth"], response_model=DefaultResponse
)
def post_change_password(user_id: str, old_password: str, new_password: str):
    was_successful = user_repository.update_user_password(
        user_id=user_id, old_password=old_password, new_password=new_password
    )
    return DefaultResponse(
        message=(
            "Password changed successfully"
            if was_successful
            else "Password was not changed"
        ),
        status="success" if was_successful else "failure",
    )
