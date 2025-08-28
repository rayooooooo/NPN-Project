from fastapi import APIRouter
from app.schemas import RouteRequest, RouteResponse, MultiRouteResponse
from app.routing.shortest_path import shortest_path, calculate_multiple_routes

router = APIRouter(prefix="/route", tags=["route"])

@router.post("", response_model=RouteResponse)
async def compute_route(req: RouteRequest):
    """Single route endpoint for backward compatibility"""
    coords, dist, t = shortest_path(req.source.lat, req.source.lon, req.target.lat, req.target.lon)
    return {"polyline": coords, "distance_m": dist, "travel_time_s": t}

@router.post("/multiple", response_model=MultiRouteResponse)
async def compute_multiple_routes(req: RouteRequest):
    """Multiple route options endpoint"""
    routes = calculate_multiple_routes(req.source.lat, req.source.lon, req.target.lat, req.target.lon)
    return {"routes": routes}
