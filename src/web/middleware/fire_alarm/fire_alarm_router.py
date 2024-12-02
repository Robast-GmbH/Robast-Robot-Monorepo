from fastapi import APIRouter
from pathlib import Path
from datetime import datetime, timedelta


fire_alarm_router = APIRouter()

DATA_FILE = Path("./fire_alarm_triggered.txt")
last_fire_alarm_publisher_poll = datetime.now()

if not DATA_FILE.exists():
    with open(DATA_FILE, "w") as f:
        f.write("false")


def read_data():
    with open(DATA_FILE, "r") as f:
        return f.read()


def write_data(data):
    with open(DATA_FILE, "w") as f:
        f.write(data)


@fire_alarm_router.get("/fire_alarm_triggered")
def get_fire_alarm_triggered(source: str | None = None):
    global last_fire_alarm_publisher_poll
    if source == "fire_alarm_publisher":
        last_fire_alarm_publisher_poll = datetime.now()
    fire_alarm_publisher_active = (
        last_fire_alarm_publisher_poll is not None
        and datetime.now() - last_fire_alarm_publisher_poll < timedelta(seconds=5)
    )
    return {
        "fire_alarm_triggered": read_data() == "True",
        "fire_alarm_publisher_active": fire_alarm_publisher_active,
    }


@fire_alarm_router.post("/fire_alarm_triggered")
def set_fire_alarm_triggered(value: bool):
    write_data(str(value))
    return {"message": f"fire_alarm_triggered updated to {value}"}
