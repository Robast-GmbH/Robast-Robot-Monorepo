from fastapi import APIRouter, HTTPException
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo
from pathlib import Path
import json

hygiene_router = APIRouter()

DEFAULT_CYCLE = 4.0
TIMEZONE = ZoneInfo("Europe/Berlin")
DATA_FILE = Path("./cleaning_data.json")

if not DATA_FILE.exists():
    with open(DATA_FILE, "w") as f:
        json.dump(
            {
                "rb_theron": {
                    "cycle": DEFAULT_CYCLE,
                    "last_cleaning": None,
                    "requires_disinfection_after_usage": False,
                }
            },
            f,
        )


def read_data():
    """Read data from the file."""
    with open(DATA_FILE, "r") as f:
        return json.load(f)


def write_data(data):
    """Write data to the file."""
    with open(DATA_FILE, "w") as f:
        json.dump(data, f)


@hygiene_router.post("/set_cycle/{robot_name}", tags=["Hygiene"])
def set_cycle(robot_name: str, cycle: float):
    """
    Set the cleaning cycle (in hours) for a specific robot.
    """
    if cycle <= 0:
        raise HTTPException(status_code=400, detail="Cycle must be a positive float.")

    data = read_data()
    if robot_name not in data:
        data[robot_name] = {
            "cycle": cycle,
            "last_cleaning": None,
            "requires_disinfection_after_usage": False,
        }

    data[robot_name]["cycle"] = cycle
    write_data(data)
    return {"message": f"Cleaning cycle updated for {robot_name}.", "cycle": cycle}


@hygiene_router.get("/get_cycle/{robot_name}", tags=["Hygiene"])
def get_cycle(robot_name: str):
    """
    Get the current cleaning cycle for a specific robot.
    """
    data = read_data()
    if robot_name not in data:
        raise HTTPException(status_code=404, detail=f"Robot '{robot_name}' not found.")

    return {"cycle": data[robot_name]["cycle"]}


@hygiene_router.post("/set_last_cleaning/{robot_name}", tags=["Hygiene"])
def set_last_cleaning(robot_name: str):
    """
    Set the last cleaning timestamp to the current time for a specific robot.
    """
    data = read_data()
    if robot_name not in data:
        data[robot_name] = {
            "cycle": DEFAULT_CYCLE,
            "last_cleaning": None,
            "requires_disinfection_after_usage": False,
        }

    data[robot_name]["last_cleaning"] = datetime.now(tz=TIMEZONE).isoformat()
    write_data(data)
    return {
        "message": f"Last cleaning timestamp updated for {robot_name}.",
        "last_cleaning": data[robot_name]["last_cleaning"],
    }


@hygiene_router.get("/get_last_cleaning/{robot_name}", tags=["Hygiene"])
def get_last_cleaning(robot_name: str):
    """
    Get the last cleaning timestamp for a specific robot.
    """
    data = read_data()
    if robot_name not in data or not data[robot_name]["last_cleaning"]:
        raise HTTPException(
            status_code=404,
            detail=f"Last cleaning timestamp not set for '{robot_name}'.",
        )

    return {"last_cleaning": data[robot_name]["last_cleaning"]}


@hygiene_router.get("/requires_cleaning/{robot_name}", tags=["Hygiene"])
def requires_cleaning(robot_name: str):
    """
    Check if cleaning is required for a specific robot based on the cycle and last cleaning timestamp.
    """
    data = read_data()
    if robot_name not in data or not data[robot_name]["last_cleaning"]:
        raise HTTPException(
            status_code=404,
            detail=f"Last cleaning timestamp not set for '{robot_name}'.",
        )

    cycle = data[robot_name]["cycle"]
    last_cleaning_time = datetime.fromisoformat(data[robot_name]["last_cleaning"])
    next_cleaning_due = last_cleaning_time + timedelta(hours=cycle)
    next_cleaning_due = next_cleaning_due.replace(tzinfo=TIMEZONE)
    now = datetime.now(tz=TIMEZONE)

    requires_cleaning = now >= next_cleaning_due
    return {
        "robot_name": robot_name,
        "requires_cleaning": requires_cleaning,
        "next_cleaning_due": next_cleaning_due.isoformat(),
        "current_time": now.isoformat(),
    }


@hygiene_router.post(
    "/set_requires_disinfection_after_usage/{robot_name}", tags=["Hygiene"]
)
def set_requires_disinfection_after_usage(
    robot_name: str, requires_disinfection_after_usage: bool
):
    """
    Set whether the robot requires disinfection after usage.
    """
    data = read_data()
    if robot_name not in data:
        data[robot_name] = {
            "cycle": DEFAULT_CYCLE,
            "last_cleaning": None,
            "requires_disinfection_after_usage": requires_disinfection_after_usage,
        }
    else:
        data[robot_name][
            "requires_disinfection_after_usage"
        ] = requires_disinfection_after_usage

    write_data(data)
    return {
        "message": f"Disinfection requirement updated for {robot_name}.",
        "requires_disinfection_after_usage": requires_disinfection_after_usage,
    }


@hygiene_router.get(
    "/get_requires_disinfection_after_usage/{robot_name}", tags=["Hygiene"]
)
def get_requires_disinfection_after_usage(robot_name: str):
    """
    Get whether the robot requires disinfection after usage.
    """
    data = read_data()
    if robot_name not in data:
        raise HTTPException(status_code=404, detail=f"Robot '{robot_name}' not found.")

    return {
        "requires_disinfection_after_usage": data[robot_name][
            "requires_disinfection_after_usage"
        ]
    }
