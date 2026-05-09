"""Cloudflare Realtime SFU API client."""

import logging

import httpx

from config import settings

log = logging.getLogger(__name__)


class CloudflareRealtimeError(Exception):
    def __init__(self, status_code: int, detail: str):
        self.status_code = status_code
        self.detail = detail
        super().__init__(f"CF Realtime API error ({status_code}): {detail}")


class CloudflareRealtime:
    """Thin client for Cloudflare Realtime SFU REST API."""

    def __init__(self) -> None:
        self.base_url = settings.cf_api_url
        self.headers = {
            "Authorization": f"Bearer {settings.cf_teleop_app_secret}",
            "Content-Type": "application/json",
        }

    async def create_session(self, sdp_offer: str) -> dict:
        """Create a new CF session. Returns sessionId + SDP answer."""
        async with httpx.AsyncClient() as client:
            resp = await client.post(
                f"{self.base_url}/sessions/new",
                headers=self.headers,
                json={"sessionDescription": {"type": "offer", "sdp": sdp_offer}},
                timeout=10.0,
            )
        if resp.status_code != 201:
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        data = resp.json()
        return {
            "cf_session_id": data["sessionId"],
            "sdp_answer": data["sessionDescription"]["sdp"],
        }

    async def add_tracks(self, session_id: str, tracks: list[dict], sdp_offer: str | None = None) -> dict:
        """Add or subscribe to tracks on a CF session."""
        body: dict = {"tracks": tracks}
        if sdp_offer:
            body["sessionDescription"] = {"type": "offer", "sdp": sdp_offer}
        async with httpx.AsyncClient() as client:
            resp = await client.post(
                f"{self.base_url}/sessions/{session_id}/tracks/new",
                headers=self.headers,
                json=body,
                timeout=10.0,
            )
        if resp.status_code not in (200, 201):
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        return resp.json()

    async def add_datachannels(self, session_id: str, channels: list[dict]) -> list[dict]:
        url = f"{self.base_url}/sessions/{session_id}/datachannels/new"
        log.info("CF add_datachannels POST %s body=%s", url, channels)
        try:
            async with httpx.AsyncClient() as client:
                resp = await client.post(
                    url,
                    headers=self.headers,
                    json={"dataChannels": channels},
                    timeout=30.0,
                )
        except httpx.HTTPError as e:
            log.error("CF add_datachannels %s failed: %r", type(e).__name__, e)
            raise
        log.info(
            "CF add_datachannels response status=%s body=%s",
            resp.status_code, resp.text[:500],
        )
        if resp.status_code not in (200, 201):
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        data = resp.json()
        if data.get("errorCode"):
            raise CloudflareRealtimeError(
                resp.status_code,
                f"{data['errorCode']}: {data.get('errorDescription', '')}",
            )
        return data.get("dataChannels", [])

    async def get_session(self, session_id: str) -> dict | None:
        """Get session info. Returns None if not found."""
        async with httpx.AsyncClient() as client:
            resp = await client.get(
                f"{self.base_url}/sessions/{session_id}",
                headers=self.headers,
                timeout=10.0,
            )
        if resp.status_code == 404:
            return None
        if resp.status_code != 200:
            raise CloudflareRealtimeError(resp.status_code, resp.text)
        return resp.json()

    async def close_session(self, session_id: str) -> None:
        """Close all tracks on a session (effectively ends it)."""
        # CF doesn't have a delete session endpoint — closing all tracks ends it.
        # Sessions auto-expire when no tracks remain.
        pass


cf_client = CloudflareRealtime()
