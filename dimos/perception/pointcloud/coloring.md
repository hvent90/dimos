# Pointcloud coloring

For every lidar frame we need the closest-in-time camera frame so that, with
intrinsics + extrinsics, we can project each point into image space and read
back a colour. Lidar runs at ~7Hz, the camera at ~14Hz, and they're captured
independently — so step one is a streaming temporal alignment.

```python session=coloring
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("hk_village1.db"))
lidar = store.streams.lidar
color_image = store.streams.color_image
print(lidar.summary())
print(color_image.summary())
```

```results
Stream("lidar"): 957 items, 2026-05-14 10:15:50 — 2026-05-14 10:18:17 (146.4s)
Stream("color_image"): 1984 items, 2026-05-14 10:15:52 — 2026-05-14 10:18:17 (144.9s)
```

`Stream.align` pairs each primary observation with the nearest one from
`other` within `tolerance` seconds. Streams iterate in ts order on both sides
and the matching is a bounded two-pointer merge — no full materialization,
no per-pair queries.

```python session=coloring
aligned = lidar.align(color_image, tolerance=0.05)
print(aligned.summary())
```

```results
Stream("lidar") | order_by(ts) -> FnIterTransformer(fn=_align): 932 items, 2026-05-14 10:15:52 — 2026-05-14 10:18:17 (144.9s)
```

Each output observation's `data` is a namedtuple keyed by source-stream name —
fully addressable both ways:

```python session=coloring
pair = aligned.first().data
print(f"lidar @ {pair.lidar.ts:.3f}  ↔  image @ {pair.color_image.ts:.3f}")
print(f"Δt = {(pair.color_image.ts - pair.lidar.ts) * 1000:.1f} ms")
print(f"index access works too: pair[0] is pair.lidar -> {pair[0] is pair.lidar}")
```

```results
lidar @ 1778753752.548  ↔  image @ 1778753752.551
Δt = 2.5 ms
index access works too: pair[0] is pair.lidar -> True
```
