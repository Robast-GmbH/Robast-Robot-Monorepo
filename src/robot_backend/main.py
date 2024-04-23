from robot_api import app
import uvicorn

if __name__ == "__main__":
    # "192.168.0.200"
    uvicorn.run(app, host="0.0.0.0", port=8005)
