# Tracks Registry

| Status | Track ID | Title | Created | Updated |
| ------ | -------- | ----- | ------- | ------- |
| [ ] | vol-abstract-iface_20260320 | Volume Abstract Interface | 2026-03-20 | 2026-03-20 |
| [ ] | vol-cache-parallel_20260320 | Volume Cache Parallel Read-Path | 2026-03-20 | 2026-03-20 |
| [ ] | vol-zarr-backend_20260320 | Zarr Volume Backend | 2026-03-20 | 2026-03-20 |
| [ ] | texturing-parallel_20260320 | Parallel PPM Generation & Texturing | 2026-03-20 | 2026-03-20 |

## Dependency Order

```
vol-abstract-iface_20260320          (no deps — start here)
    ├── vol-cache-parallel_20260320  (depends on abstract iface)
    │       └── texturing-parallel_20260320  (depends on both above)
    └── vol-zarr-backend_20260320    (depends on abstract iface; prefetch phase also needs cache track)
```
