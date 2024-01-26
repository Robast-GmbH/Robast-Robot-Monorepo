# connection interface between the fleet adapter and the fleet manager
from fastapi import Depends, FastAPI
from typing import Optional
import socket



app = FastAPI()

@app.post("/navigate")
def navigate():
    # Implement your logic here
    pass

@app.post("/start_activity")
def start_activity():
    # Implement your logic here
    pass

@app.post("/stop")
def stop():
    # Implement your logic here
    pass

@app.get("/position")
def get_position():
    # Implement your logic here
    pass

@app.get("/map")
def get_map():
    # Implement your logic here
    pass

@app.get("/battery_soc")
def get_battery_soc():
    # Implement your logic here
    pass

@app.get("/is_command_completed")
def is_command_completed():
    # Implement your logic here
    pass

def find_available_port(start_port: int) -> Optional[int]:
    for port in range(start_port, 65536):  # The maximum port number is 65535
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("0.0.0.0", port))
                return port
            except OSError:
                pass
    return None

def api_main():
    import uvicorn
    port = find_available_port(8000)
    if port is not None:
        uvicorn.run(app, host="0.0.0.0", port=port)
    else:
        print("No available port found.")

if __name__ == "__main__":
    api_main()
