import uvicorn
from middleware_api import app
import os

if __name__ == "__main__":
    home_directory = os.path.expanduser("~")
    os.makedirs(f"{home_directory}/databases", exist_ok=True)
    os.makedirs(f"{home_directory}/log", exist_ok=True)
    os.makedirs(f"{home_directory}/uploaded_files", exist_ok=True)
    uvicorn.run(app, port=8003, host="0.0.0.0")
