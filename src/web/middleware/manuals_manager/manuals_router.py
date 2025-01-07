from fastapi import APIRouter, File, UploadFile, HTTPException
from fastapi.responses import FileResponse
import os
from pathlib import Path
from models.logger import Logger


manuals_router = APIRouter()
manuals_logger = Logger("manuals_manager", "log/manuals_manager.log")

home_directory= os.path.expanduser("~")
UPLOAD_FOLDER = f"{home_directory}/uploaded_files"
BASE_DIR = Path(UPLOAD_FOLDER).resolve()


def get_file_path(file_name: str) -> Path:
    file_path = BASE_DIR / file_name
    if not file_path.resolve().is_relative_to(BASE_DIR):
        raise ValueError("Attempted path traversal detected")
    if not file_name.endswith(".pdf"):
        raise ValueError("Invalid file type")
    return Path(file_path)


@manuals_router.post("/", tags=["Manuals"])
async def upload_or_update_pdf(file: UploadFile = File(...)):
    """
    Create or update a PDF file by uploading it.
    If the file already exists, it will be overwritten.
    """
    if file.content_type != "application/pdf":
        raise HTTPException(
            status_code=400, detail="Invalid file type. Only PDF files are allowed."
        )

    file_location = get_file_path(file.filename)
    action = "updated" if file_location.exists() else "uploaded"

    with open(file_location, "wb") as f:
        f.write(await file.read())

    manuals_logger.info(f"{action} '{file.filename}'.")

    return {
        "message": f"File '{file.filename}' {action} successfully.",
        "path": str(file_location),
    }


@manuals_router.get("/", tags=["Manuals"])
async def list_files():
    """
    List all uploaded PDF files.
    """
    files = [
        {"name": f, "last_modified": Path(get_file_path(f)).stat().st_mtime}
        for f in os.listdir(UPLOAD_FOLDER)
        if os.path.isfile(get_file_path(f))
    ]
    return {"files": files}


@manuals_router.get("/{file_name}", tags=["Manuals"])
async def get_pdf(file_name: str):
    """
    Retrieve a specific PDF file by name.
    """
    file_path = get_file_path(file_name)
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="File not found.")

    return FileResponse(file_path, media_type="application/pdf", filename=file_name)


@manuals_router.delete("/{file_name}", tags=["Manuals"])
async def delete_pdf(file_name: str):
    """
    Delete a specific PDF file by name.
    """
    file_path = get_file_path(file_name)
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="File not found.")

    manuals_logger.info(f"Deleted '{file_name}'.")

    os.remove(file_path)
    return {"message": f"File '{file_name}' deleted successfully."}


@manuals_router.delete("/", tags=["Manuals"])
async def delete_all_files():
    """
    Delete all uploaded PDF files.
    """
    files_deleted = 0
    for file_name in os.listdir(UPLOAD_FOLDER):
        file_path = get_file_path(file_name)
        if file_path.is_file():
            os.remove(file_path)
            files_deleted += 1

    manuals_logger.info(f"Deleted {files_deleted} file(s).")

    return {"message": f"Deleted {files_deleted} file(s)."}
