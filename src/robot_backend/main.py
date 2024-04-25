from robot_api import app
import uvicorn

if __name__ == "__main__":
    uvicorn.run(app, host="192.168.0.200", port=8001)