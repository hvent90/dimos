"""Session lifecycle endpoints."""

from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from models.database import get_db
from models.session import TeleopSession
from services.auth import get_current_user, get_robot_id
from services.cloudflare import CloudflareRealtimeError, cf_client

router = APIRouter(prefix="/sessions", tags=["sessions"])


# ─── Request/Response schemas ────────────────────────────────────────


class CreateSessionRequest(BaseModel):
    robot_id: str
    robot_name: str
    sdp_offer: str


class CreateSessionResponse(BaseModel):
    session_id: str
    cf_session_id: str
    sdp_answer: str
    ice_servers: list[dict]


class JoinSessionRequest(BaseModel):
    role: str = "operator"  # operator | viewer
    sdp_offer: str


class JoinSessionResponse(BaseModel):
    cf_session_id: str
    sdp_answer: str
    robot_cf_session_id: str
    ice_servers: list[dict]
    role: str


class HeartbeatRequest(BaseModel):
    rtt_ms: float | None = None
    packet_loss_pct: float | None = None
    video_bitrate_kbps: int | None = None
    command_rate_hz: float | None = None
    safety_state: str = "nominal"


class SessionInfo(BaseModel):
    session_id: str
    robot_id: str
    robot_name: str
    state: str
    operator_id: str | None
    rtt_ms: float | None
    packet_loss_pct: float | None
    created_at: datetime


class LeaveRequest(BaseModel):
    reason: str = "user_initiated"


ICE_SERVERS = [{"urls": "stun:stun.cloudflare.com:3478"}]


# ─── Robot endpoints ─────────────────────────────────────────────────


@router.post("", response_model=CreateSessionResponse, status_code=201)
async def create_session(
    body: CreateSessionRequest,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot registers itself. Creates Cloudflare SFU session."""
    # Verify robot_id matches the authenticated key
    if body.robot_id != robot_id:
        raise HTTPException(status_code=403, detail="Robot ID mismatch")

    # Close existing session for this robot if any
    existing = await db.execute(
        select(TeleopSession).where(
            TeleopSession.robot_id == robot_id,
            TeleopSession.state != "disconnected",
        )
    )
    for old in existing.scalars():
        old.state = "disconnected"

    # Create CF session
    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")

    # Store session
    session = TeleopSession(
        robot_id=robot_id,
        robot_name=body.robot_name,
        state="idle",
        cf_session_id=cf_result["cf_session_id"],
    )
    db.add(session)
    await db.commit()
    await db.refresh(session)

    return CreateSessionResponse(
        session_id=session.id,
        cf_session_id=cf_result["cf_session_id"],
        sdp_answer=cf_result["sdp_answer"],
        ice_servers=ICE_SERVERS,
    )


@router.post("/{session_id}/heartbeat")
async def heartbeat(
    session_id: str,
    body: HeartbeatRequest,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot reports connection quality metrics."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.robot_id != robot_id:
        raise HTTPException(status_code=404, detail="Session not found")

    session.rtt_ms = body.rtt_ms
    session.packet_loss_pct = body.packet_loss_pct
    session.video_bitrate_kbps = body.video_bitrate_kbps
    session.command_rate_hz = body.command_rate_hz
    session.last_heartbeat = datetime.now(timezone.utc)
    await db.commit()

    return {"ack": True}


@router.delete("/{session_id}", status_code=204)
async def delete_session(
    session_id: str,
    robot_id: str = Depends(get_robot_id),
    db: AsyncSession = Depends(get_db),
):
    """Robot going offline."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.robot_id != robot_id:
        raise HTTPException(status_code=404, detail="Session not found")

    session.state = "disconnected"
    await db.commit()


# ─── Operator endpoints ──────────────────────────────────────────────


@router.get("", response_model=list[SessionInfo])
async def list_sessions(
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """List available robots (active sessions)."""
    result = await db.execute(
        select(TeleopSession).where(TeleopSession.state.in_(["idle", "active"]))
    )
    sessions = result.scalars().all()
    return [
        SessionInfo(
            session_id=s.id,
            robot_id=s.robot_id,
            robot_name=s.robot_name,
            state=s.state,
            operator_id=s.operator_id,
            rtt_ms=s.rtt_ms,
            packet_loss_pct=s.packet_loss_pct,
            created_at=s.created_at,
        )
        for s in sessions
    ]


@router.post("/{session_id}/join", response_model=JoinSessionResponse)
async def join_session(
    session_id: str,
    body: JoinSessionRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator or viewer joins a session."""
    session = await db.get(TeleopSession, session_id)
    if not session or session.state == "disconnected":
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    # Enforce single operator
    if body.role == "operator":
        if session.operator_id and session.operator_id != user_id:
            raise HTTPException(
                status_code=409,
                detail=f"Session already has operator: {session.operator_id}",
            )

    # Create a new CF session for this operator (they get their own PeerConnection)
    try:
        cf_result = await cf_client.create_session(body.sdp_offer)
    except CloudflareRealtimeError as e:
        raise HTTPException(status_code=502, detail=f"Cloudflare error: {e.detail}")

    # Update session state
    if body.role == "operator":
        session.operator_id = user_id
        session.operator_cf_session_id = cf_result["cf_session_id"]
        session.state = "active"
        await db.commit()

    return JoinSessionResponse(
        cf_session_id=cf_result["cf_session_id"],
        sdp_answer=cf_result["sdp_answer"],
        robot_cf_session_id=session.cf_session_id,
        ice_servers=ICE_SERVERS,
        role=body.role,
    )


@router.post("/{session_id}/leave")
async def leave_session(
    session_id: str,
    body: LeaveRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Operator or viewer leaves."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    user_id = user["sub"]

    if session.operator_id == user_id:
        session.operator_id = None
        session.operator_cf_session_id = None
        session.state = "idle"
        await db.commit()

    return {"session_id": session_id, "state": session.state}


@router.get("/{session_id}/status", response_model=SessionInfo)
async def session_status(
    session_id: str,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Get session status and connection quality."""
    session = await db.get(TeleopSession, session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    return SessionInfo(
        session_id=session.id,
        robot_id=session.robot_id,
        robot_name=session.robot_name,
        state=session.state,
        operator_id=session.operator_id,
        rtt_ms=session.rtt_ms,
        packet_loss_pct=session.packet_loss_pct,
        created_at=session.created_at,
    )
