"""SDP introspection helpers — minimal regex-based extraction.

Robust against optional attribute ordering; not a full SDP parser. Only
covers what the broker needs: finding the first publishable m=video section
in a peer's offer so we can register it with CF Realtime's
`/sessions/{id}/tracks/new` endpoint.
"""

from __future__ import annotations

import re

_MID_RE = re.compile(r"^a=mid:(\S+)", re.MULTILINE)
_MSID_RE = re.compile(r"^a=msid:(\S+)\s+(\S+)", re.MULTILINE)


def extract_video_track(sdp: str) -> tuple[str, str] | None:
    """Return (mid, track_name) for the first sendonly/sendrecv m=video.

    Returns None if the offer has no such section, if every m=video is
    recvonly/inactive, or if the matching section lacks an `a=mid` or
    `a=msid` line. ``track_name`` is the *second* token of
    ``a=msid:<stream> <track>`` — aiortc emits both.

    Examples:
        >>> sdp = (
        ...     "v=0\\r\\n"
        ...     "m=video 9 UDP/TLS/RTP/SAVPF 96\\r\\n"
        ...     "a=mid:1\\r\\n"
        ...     "a=msid:- robot-video\\r\\n"
        ...     "a=sendonly\\r\\n"
        ... )
        >>> extract_video_track(sdp)
        ('1', 'robot-video')
    """
    in_video = False
    is_send = False
    section: list[str] = []

    def _flush() -> tuple[str, str] | None:
        if not (in_video and is_send):
            return None
        joined = "\n".join(section)
        mid_m = _MID_RE.search(joined)
        msid_m = _MSID_RE.search(joined)
        if mid_m and msid_m:
            return mid_m.group(1), msid_m.group(2)
        return None

    for line in sdp.splitlines():
        if line.startswith("m="):
            found = _flush()
            if found is not None:
                return found
            in_video = line.startswith("m=video")
            is_send = False
            section = [line]
            continue
        section.append(line)
        if in_video and (
            line.startswith("a=sendonly") or line.startswith("a=sendrecv")
        ):
            is_send = True

    return _flush()
