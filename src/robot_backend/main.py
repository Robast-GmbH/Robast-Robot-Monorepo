from robot_api import app
import uvicorn

if __name__ == "__main__":
    # host="192.168.0.200" on robot "0.0.0.0" for localhost
    uvicorn.run(app, host="0.0.0.0", port=8001)
