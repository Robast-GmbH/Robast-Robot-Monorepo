from dataclasses import dataclass
from typing import List, Dict, Any


@dataclass
class Vertice:
    x: float
    y: float
    name: str
    is_dispenser: bool = False
    is_ingestor: bool = False

    @classmethod
    def from_json(cls, json: Dict[str, Any]) -> "Vertice":
        params = cls._read_params(json.get("params", []))
        return cls(
            x=json.get("x", 0.0),
            y=json.get("y", 0.0),
            name=json.get("name", ""),
            is_dispenser=params.get("pickup_dispenser", False),
            is_ingestor=params.get("dropoff_ingestor", False),
        )

    @staticmethod
    def _read_params(params: List[Dict[str, Any]]) -> Dict[str, bool]:
        vertices_params = {}
        for param in params:
            vertices_params[param.get("name", "")] = True
        return vertices_params


@dataclass
class Level:
    name: str
    vertices: List[Vertice]

    @classmethod
    def from_json(cls, json: Dict[str, Any]) -> "Level":
        return cls(
            name=json.get("name", ""),
            vertices=[
                Vertice.from_json(v)
                for v in json.get("nav_graphs", [{}])[0].get("vertices", [])
            ],
        )


@dataclass
class BuildingMap:
    name: str
    levels: List[Level]

    @classmethod
    def from_json(cls, json: Dict[str, Any]) -> "BuildingMap":
        return cls(
            name=json.get("name", ""),
            levels=[Level.from_json(l) for l in json.get("levels", [])],
        )
