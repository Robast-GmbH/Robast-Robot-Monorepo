from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
import os
from pathlib import Path

log_router = APIRouter()

DOWNLOAD_FOLDER = "./log"
AVAILABLE_LOGS = [
    "fire_alarm.log",
    "hygiene_manager.log",
    "manuals_manager.log",
    "module_manager.log",
    "user_system.log",
]

os.makedirs(DOWNLOAD_FOLDER, exist_ok=True)


@log_router.get("/", tags=["Logs"])
def get_available_logs():
    return {"logs": AVAILABLE_LOGS}


@log_router.get("/{log_name}", tags=["Logs"])
def download_log(log_name: str):
    if log_name not in AVAILABLE_LOGS:
        raise HTTPException(status_code=404, detail="Log not found.")
    return FileResponse(Path(f"{DOWNLOAD_FOLDER}/{log_name}"), filename=log_name)
