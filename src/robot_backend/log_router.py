from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import FileResponse
from pathlib import Path

log_router = APIRouter()

LOG_FOLDER = Path("./log").resolve()

available_logs = [
    str(file.relative_to(LOG_FOLDER))
    for file in LOG_FOLDER.rglob("*")
    if file.is_file()
]


def get_file_path(file_name: str) -> Path:
    file_path = LOG_FOLDER / file_name
    if not file_path.resolve().is_relative_to(LOG_FOLDER):
        raise ValueError("Attempted path traversal detected")
    if not file_name.endswith(".log"):
        raise ValueError("Invalid file type")
    return file_path


@log_router.get("/", tags=["Logs"])
def get_available_logs():
    return {"logs": available_logs}


@log_router.post("/download", tags=["Logs"])
def download_log(log_name: str = Body(...)):
    if log_name not in available_logs:
        raise HTTPException(status_code=404, detail="Log not found.")
    file_path = get_file_path(log_name)
    return FileResponse(file_path, filename=log_name)
