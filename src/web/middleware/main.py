import uvicorn
from middleware_api import app

if __name__ == "__main__":
    uvicorn.run(app, port=8003, host="0.0.0.0")
