"""JWT auth for operators and API key auth for robots."""

from datetime import datetime, timedelta, timezone

from fastapi import Depends, HTTPException, Security
from fastapi.security import APIKeyHeader, HTTPAuthorizationCredentials, HTTPBearer
from jose import JWTError, jwt
from passlib.context import CryptContext

from config import settings

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
bearer_scheme = HTTPBearer(auto_error=False)
api_key_header = APIKeyHeader(name="X-Robot-API-Key", auto_error=False)

# In-memory robot API keys (move to DB for production)
# Format: { "key": "robot_id" }
ROBOT_API_KEYS: dict[str, str] = {}


def hash_password(password: str) -> str:
    return pwd_context.hash(password)


def verify_password(plain: str, hashed: str) -> bool:
    return pwd_context.verify(plain, hashed)


def create_token(user_id: str, role: str = "operator") -> str:
    expire = datetime.now(timezone.utc) + timedelta(hours=settings.jwt_expire_hours)
    payload = {"sub": user_id, "role": role, "exp": expire}
    return jwt.encode(payload, settings.jwt_secret, algorithm=settings.jwt_algorithm)


def decode_token(token: str) -> dict:
    try:
        return jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
    except JWTError as e:
        raise HTTPException(status_code=401, detail=f"Invalid token: {e}")


async def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> dict:
    """Extract user from Bearer JWT token."""
    if credentials is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return decode_token(credentials.credentials)


async def get_robot_id(
    api_key: str | None = Security(api_key_header),
    credentials: HTTPAuthorizationCredentials | None = Security(bearer_scheme),
) -> str:
    """Authenticate robot via API key header or Bearer token."""
    # Try API key first
    if api_key and api_key in ROBOT_API_KEYS:
        return ROBOT_API_KEYS[api_key]

    # Fall back to Bearer token
    if credentials:
        payload = decode_token(credentials.credentials)
        if payload.get("role") == "robot":
            return payload["sub"]

    raise HTTPException(status_code=401, detail="Invalid robot credentials")


def register_robot_key(api_key: str, robot_id: str) -> None:
    """Register an API key for a robot. Call from admin endpoint or startup."""
    ROBOT_API_KEYS[api_key] = robot_id
