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

"""Live CLI dashboard for pub/sub traffic over the active transport (LCM or Zenoh)."""

from __future__ import annotations

from rich.text import Text
from textual.app import App, ComposeResult
from textual.color import Color
from textual.widgets import DataTable

from dimos.utils.cli import theme
from dimos.utils.cli.spy.spy import Spy


def gradient(max_value: float, value: float) -> str:
    """Gradient from cyan (low) to yellow (high) using DimOS theme colors."""
    ratio = min(value / max_value, 1.0)
    cyan = Color.parse(theme.CYAN)
    yellow = Color.parse(theme.YELLOW)
    return cyan.blend(yellow, ratio).hex


def topic_text(topic_name: str) -> Text:
    """Format a topic/key name, highlighting the type suffix.

    LCM channels use ``/topic#pkg.Msg``; Zenoh keys embed the type as the last
    ``/`` segment (e.g. ``dimos/cmd_vel#geometry_msgs.Twist`` after decoding,
    or ``dimos/cmd_vel/geometry_msgs.Twist`` for an unresolved key).
    """
    if "#" in topic_name:
        base, suffix = topic_name.split("#", 1)
        return Text(base, style=theme.BRIGHT_WHITE) + Text("#" + suffix, style=theme.BLUE)
    if topic_name.startswith("/rpc"):
        return Text("/rpc", style=theme.BLUE) + Text(topic_name[4:], style=theme.BRIGHT_WHITE)
    return Text(topic_name, style=theme.BRIGHT_WHITE)


class SpyApp(App):  # type: ignore[type-arg]
    """Real-time CLI dashboard for pub/sub traffic statistics using Textual."""

    CSS_PATH = "../dimos.tcss"

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    DataTable {{
        height: 2fr;
        width: 1fr;
        border: solid {theme.BORDER};
        background: {theme.BG};
        scrollbar-size: 0 0;
    }}
    DataTable > .datatable--header {{
        color: {theme.ACCENT};
        background: transparent;
    }}
    """

    refresh_interval: float = 0.5

    BINDINGS = [
        ("q", "quit"),
        ("ctrl+c", "quit"),
    ]

    def __init__(
        self,
        transport: str | None = None,
        key: str | None = None,
        connect: list[str] | None = None,
        iface: str | None = None,
        *args,  # type: ignore[no-untyped-def]
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        # Warn about missing system config before entering TUI raw mode (LCM only).
        if (transport or "").lower() != "zenoh":
            from dimos.protocol.service.lcmservice import autoconf

            autoconf(check_only=True)

        self.spy = Spy(
            transport=transport, key=key, connect=connect, iface=iface, graph_log_window=0.5
        )
        self.spy.start()
        self.table: DataTable | None = None  # type: ignore[type-arg]

    def compose(self) -> ComposeResult:
        self.table = DataTable(zebra_stripes=False, cursor_type=None)  # type: ignore[arg-type]
        self.table.add_column("Topic")
        self.table.add_column("Freq (Hz)")
        self.table.add_column("Bandwidth")
        self.table.add_column("Total Traffic")
        yield self.table

    def on_mount(self) -> None:
        self.set_interval(self.refresh_interval, self.refresh_table)

    async def on_unmount(self) -> None:
        self.spy.stop()

    def refresh_table(self) -> None:
        topics = list(self.spy.topic.values())
        topics.sort(key=lambda t: t.total_traffic(), reverse=True)
        self.table.clear(columns=False)  # type: ignore[union-attr]

        for t in topics:
            freq = t.freq(5.0)
            kbps = t.kbps(5.0)
            self.table.add_row(  # type: ignore[union-attr]
                topic_text(t.name),
                Text(f"{freq:.1f}", style=gradient(10, freq)),
                Text(t.kbps_hr(5.0), style=gradient(1024 * 3, kbps)),
                Text(t.total_traffic_hr()),
            )


def run_noninteractive(
    transport: str | None,
    key: str | None,
    interval: float,
    duration: float | None,
    connect: list[str] | None = None,
    iface: str | None = None,
) -> None:
    """Print a periodic plain-text traffic snapshot to stdout (no TUI).

    Used for non-interactive shells, piping, and logging. Stops on Ctrl+C or
    after ``duration`` seconds (if given).
    """
    import sys
    import time

    from dimos.core.global_config import global_config

    resolved = transport or global_config.transport

    if resolved != "zenoh":
        from dimos.protocol.service.lcmservice import autoconf

        autoconf(check_only=True)

    spy = Spy(transport=transport, key=key, connect=connect, iface=iface, graph_log_window=interval)
    spy.start()
    deadline = None if duration is None else time.monotonic() + duration
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(interval)
            topics = sorted(spy.topic.values(), key=lambda t: t.total_traffic(), reverse=True)
            print(f"--- {len(topics)} topic(s) (transport={resolved}) ---")
            for t in topics:
                print(
                    f"  {t.name:55s} {t.freq(5.0):7.1f} Hz  "
                    f"{t.kbps_hr(5.0):>12s}  total {t.total_traffic_hr()}"
                )
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        spy.stop()


def main() -> None:
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        prog="spy", description="Live TUI for pub/sub traffic (LCM or Zenoh)."
    )
    parser.add_argument(
        "--transport",
        choices=["lcm", "zenoh"],
        help="Transport backend (defaults to DIMOS_TRANSPORT / .env).",
    )
    parser.add_argument(
        "--key",
        help="Override the wildcard: LCM channel regex (e.g. '/odom.*') "
        "or Zenoh key expr (e.g. 'dimos/**').",
    )
    parser.add_argument(
        "--connect",
        action="append",
        metavar="ENDPOINT",
        help="Zenoh endpoint to dial, e.g. 'tcp/10.21.31.106:7447'. Repeatable. "
        "Needed to reach a peer with scouting disabled.",
    )
    parser.add_argument(
        "--iface",
        metavar="NIC",
        help="Pin the Zenoh multicast scout interface, e.g. 'eth0' (Zenoh only). "
        "Defaults to DIMOS_ZENOH_IFACE.",
    )
    parser.add_argument(
        "-n",
        "--noninteractive",
        action="store_true",
        help="Print plain-text snapshots instead of the TUI "
        "(auto-enabled when stdout is not a terminal).",
    )
    parser.add_argument(
        "--interval", type=float, default=1.0, help="Snapshot interval, seconds (noninteractive)."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Exit after this many seconds (noninteractive). Default: run until Ctrl+C.",
    )
    args = parser.parse_args()

    noninteractive = args.noninteractive or not sys.stdout.isatty()
    if noninteractive:
        run_noninteractive(
            args.transport,
            args.key,
            args.interval,
            args.duration,
            connect=args.connect,
            iface=args.iface,
        )
    else:
        SpyApp(
            transport=args.transport, key=args.key, connect=args.connect, iface=args.iface
        ).run()


if __name__ == "__main__":
    main()
