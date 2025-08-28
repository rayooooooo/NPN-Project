from fastapi import APIRouter
from sqlalchemy import text
from app.database import engine

router = APIRouter(prefix="/vehicles", tags=["vehicles"])

@router.get("")
async def list_live_positions():
    q = text(
        """
        SELECT vehicle_id,
               ST_Y(ST_AsText((geom::geometry))) as lat,
               ST_X(ST_AsText((geom::geometry))) as lon,
               speed_kph,
               ts
        FROM (
          SELECT vehicle_id, geom, speed_kph, ts,
                 ROW_NUMBER() OVER (PARTITION BY vehicle_id ORDER BY ts DESC) rn
          FROM vehicle_positions
        ) t
        WHERE rn=1
        ORDER BY vehicle_id
        """
    )
    with engine.connect() as conn:
        rows = conn.execute(q).mappings().all()
    return {"vehicles": [dict(r) for r in rows]}
