"""Cognito JWT auth for operators and API key auth for robots.

Operators sign in against the Cognito user pool from the SPA; this module
only *verifies* the resulting RS256 ID tokens against the pool's JWKS.
The app-level identity is the (verified) email claim, so API key ownership,
robot-ID namespacing and session ownership are all keyed by email.
"""

import asyncio
import logging
import time

from config import settings
from fastapi import HTTPException, Security
from fastapi.security import APIKeyHeader, HTTPAuthorizationCredentials, HTTPBearer
import httpx
from jose import JWTError, jwt

log = logging.getLogger(__name__)

bearer_scheme = HTTPBearer(auto_error=False)
api_key_header = APIKeyHeader(name="X-Robot-API-Key", auto_error=False)

# In-memory robot API keys (dev bootstrap only — prod uses dtk_* keys in DB)
ROBOT_API_KEYS: dict[str, str] = {}

# --- Cognito JWKS cache. Hourly TTL so a key rotation doesn't need a restart.
_JWKS_TTL_SEC = 3600

_jwks_lock = asyncio.Lock()
_jwks_keys: dict[str, dict] = {}  # kid -> JWK
_jwks_fetched_at = 0.0


async def _fetch_jwks() -> dict[str, dict]:
    url = f"{settings.cognito_issuer}/.well-known/jwks.json"
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.get(url)
    resp.raise_for_status()
    return {k["kid"]: k for k in resp.json()["keys"]}


async def _get_jwk(kid: str) -> dict | None:
    # Fast path: cached + fresh, no lock or I/O.
    global _jwks_fetched_at
    now = time.monotonic()
    if kid in _jwks_keys and now - _jwks_fetched_at < _JWKS_TTL_SEC:
        return _jwks_keys[kid]
    # Lock collapses concurrent misses into one fetch.
    async with _jwks_lock:
        if kid in _jwks_keys and time.monotonic() - _jwks_fetched_at < _JWKS_TTL_SEC:
            return _jwks_keys[kid]
        try:
            _jwks_keys.update(await _fetch_jwks())
            _jwks_fetched_at = time.monotonic()
        except Exception as e:
            log.error("JWKS fetch failed: %s", e)
    return _jwks_keys.get(kid)


async def decode_token(token: str) -> dict:
    """Verify a Cognito ID token and return our app-level claims.

    The returned dict keeps the historical shape: `sub` is the user's email
    (everything downstream keys on it) and `role` is operator/admin.
    """
    try:
        kid = jwt.get_unverified_header(token).get("kid", "")
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")

    jwk = await _get_jwk(kid)
    if jwk is None:
        raise HTTPException(status_code=401, detail="Invalid token: unknown key id")

    try:
        claims = jwt.decode(
            token,
            jwk,
            algorithms=["RS256"],
            audience=settings.cognito_client_id,
            issuer=settings.cognito_issuer,
        )
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")

    if claims.get("token_use") != "id":
        raise HTTPException(status_code=401, detail="Invalid token: not an ID token")
    email = claims.get("email")
    if not email or not claims.get("email_verified"):
        raise HTTPException(status_code=401, detail="Email not verified")

    role = "admin" if "admin" in claims.get("cognito:groups", []) else "operator"
    return {"sub": email, "role": role, "cognito_sub": claims["sub"]}


async def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> dict:
    """Extract user from Bearer Cognito ID token."""
    if credentials is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return await decode_token(credentials.credentials)


async def get_robot_id(api_key: str | None = Security(api_key_header)) -> str:
    """Authenticate robot via X-Robot-API-Key header."""
    if api_key:
        # Dev bootstrap keys first
        if api_key in ROBOT_API_KEYS:
            return ROBOT_API_KEYS[api_key]

        # DB-backed API keys (dtk_live_... / dtk_dev_...)
        from models.database import async_session

        from services.keys import validate_api_key

        async with async_session() as db:
            key_record = await validate_api_key(db, api_key)
            if key_record:
                if not key_record.robot_id:
                    # Multiple such keys share owner_id as identity; ops should fix.
                    log.warning(
                        "API key %s... has no robot_id; using owner_id as identity",
                        api_key[:12],
                    )
                return key_record.robot_id or key_record.owner_id

    raise HTTPException(status_code=401, detail="Invalid robot credentials")


async def get_robot_owner(api_key: str | None = Security(api_key_header)) -> str:
    """Authenticate robot via X-Robot-API-Key; return the key's owner (tenant)."""
    if api_key:
        if api_key in ROBOT_API_KEYS:
            return ROBOT_API_KEYS[api_key]  # dev bootstrap: value doubles as owner
        from models.database import async_session

        from services.keys import validate_api_key

        async with async_session() as db:
            key_record = await validate_api_key(db, api_key)
            if key_record:
                return key_record.owner_id

    raise HTTPException(status_code=401, detail="Invalid robot credentials")


async def get_operator_or_robot(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
    api_key: str | None = Security(api_key_header),
) -> dict:
    """Accept either identity for endpoints both sides use (TURN credentials).

    Robot key wins when both are present (robots never send a Bearer token).
    """
    if api_key:
        robot_id = await get_robot_id(api_key)
        return {"sub": robot_id, "role": "robot"}
    if credentials is not None:
        return await decode_token(credentials.credentials)
    raise HTTPException(status_code=401, detail="Not authenticated")


def register_robot_key(api_key: str, robot_id: str) -> None:
    """Register an in-memory API key for a robot (dev bootstrap)."""
    ROBOT_API_KEYS[api_key] = robot_id
