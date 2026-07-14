# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Passive-first rate limiting (HARDENING_PLAN C2).

Token buckets keyed by (caller, route class), applied as ASGI middleware.
PASSIVE by default: when a bucket runs dry we log the would-be 429 and let
the request through — flip RATE_LIMIT_ENFORCE=true once a week of logs shows
the limits don't bite legitimate traffic.

Only sensitive routes are classed; heartbeats and everything unlisted are
exempt (they're high-frequency by design and self-limit via session auth).

No external deps (repo stays requirements-light); pure logic lives in
TokenBucket so tests can drive it with a fake clock.
"""

import hashlib
import logging
import time

from fastapi import Request
from fastapi.responses import JSONResponse
from metrics import RATE_LIMIT_HITS

log = logging.getLogger(__name__)

# route class → (max tokens = burst, refill tokens/sec). Rates are per caller.
LIMITS: dict[str, tuple[float, float]] = {
    "keys_write": (10, 10 / 60),  # key mint/revoke: 10/min
    "session_join": (12, 12 / 60),  # operator join + robot create: 12/min
    "turn_credentials": (6, 6 / 60),  # each mint hits the CF TURN API
}


def classify(method: str, path: str) -> str | None:
    """Map a request to a route class; None = unlimited."""
    if not path.startswith("/api/v1/"):
        return None
    p = path[len("/api/v1") :]
    if p.startswith("/keys") and method in ("POST", "DELETE"):
        return "keys_write"
    if p == "/sessions/turn-credentials":
        return "turn_credentials"
    if method == "POST" and (p == "/sessions" or p.endswith("/join")):
        return "session_join"
    return None  # heartbeats, lists, leave, bridge, … unlimited


class TokenBucket:
    """Classic token bucket; clock injectable for tests."""

    def __init__(self, capacity: float, refill_per_sec: float, now: float) -> None:
        self.capacity = capacity
        self.refill = refill_per_sec
        self.tokens = capacity
        self.stamp = now

    def allow(self, now: float) -> bool:
        self.tokens = min(self.capacity, self.tokens + (now - self.stamp) * self.refill)
        self.stamp = now
        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True
        return False

    def retry_after_sec(self) -> int:
        deficit = 1.0 - self.tokens
        return max(1, int(deficit / self.refill) + 1)


class RateLimiter:
    """(caller, class) → TokenBucket, with size-bounded pruning."""

    _MAX_BUCKETS = 10_000  # ~100B each; prune idle ones past this

    def __init__(self, enforce: bool, clock=time.monotonic) -> None:
        self.enforce = enforce
        self.clock = clock
        self.buckets: dict[tuple[str, str], TokenBucket] = {}

    def check(self, caller: str, route_class: str) -> tuple[bool, int]:
        """Returns (allowed, retry_after_sec). Passive mode still counts, so
        the logs show exactly what enforcement would have blocked."""
        now = self.clock()
        key = (caller, route_class)
        bucket = self.buckets.get(key)
        if bucket is None:
            if len(self.buckets) >= self._MAX_BUCKETS:
                self._prune(now)
            capacity, refill = LIMITS[route_class]
            bucket = self.buckets[key] = TokenBucket(capacity, refill, now)
        allowed = bucket.allow(now)
        return allowed, 0 if allowed else bucket.retry_after_sec()

    def _prune(self, now: float) -> None:
        # Drop buckets idle long enough to be full again — denying them is
        # impossible, so forgetting them is lossless.
        idle_cutoff = max(cap / refill for cap, refill in LIMITS.values())
        stale = [k for k, b in self.buckets.items() if now - b.stamp > idle_cutoff]
        for k in stale:
            del self.buckets[k]


def _fp(secret: str) -> str:
    # Non-reversible 16-hex fingerprint: stable per caller, safe to log.
    return hashlib.sha256(secret.encode("utf-8")).hexdigest()[:16]


def caller_id(request: Request) -> str:
    """Caller identity for bucketing (auth runs later, so no verified subject).
    Keys/tokens are fingerprinted — never logged raw; XFF (Caddy) covers probing.
    """
    robot_key = request.headers.get("x-robot-api-key")
    if robot_key:
        return f"key:{_fp(robot_key)}"
    auth = request.headers.get("authorization", "")
    if auth.startswith("Bearer ") and len(auth) > 7:
        return f"tok:{_fp(auth[7:])}"
    fwd = request.headers.get("x-forwarded-for", "")
    ip = fwd.split(",")[0].strip() if fwd else (request.client.host if request.client else "?")
    return f"ip:{ip}"


def install(app, enforce: bool) -> RateLimiter:
    """Attach the middleware; returns the limiter (tests poke its clock)."""
    limiter = RateLimiter(enforce=enforce)

    @app.middleware("http")
    async def _rate_limit(request: Request, call_next):
        route_class = classify(request.method, request.url.path)
        if route_class is None:
            return await call_next(request)
        caller = caller_id(request)
        allowed, retry_after = limiter.check(caller, route_class)
        if not allowed:
            RATE_LIMIT_HITS.labels(route_class, str(limiter.enforce).lower()).inc()
            if limiter.enforce:
                return JSONResponse(
                    status_code=429,
                    content={"detail": "Rate limit exceeded"},
                    headers={"Retry-After": str(retry_after)},
                )
            log.warning(
                "rate-limit (passive, would 429): caller=%s class=%s %s %s",
                caller,
                route_class,
                request.method,
                request.url.path,
            )
        return await call_next(request)

    return limiter
