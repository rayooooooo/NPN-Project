from fastapi import APIRouter
from app.schemas import OptimizeRequest, OptimizeResponse
from app.optimization.vrp import solve_vrp

router = APIRouter(prefix="/optimize", tags=["optimize"])

@router.post("", response_model=OptimizeResponse)
async def optimize(req: OptimizeRequest):
    vehicles = [v.dict() for v in req.vehicles]
    stops = [s.dict() for s in req.stops]
    result = solve_vrp(vehicles, stops)
    return result
