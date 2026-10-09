# Godot references

Source and executable: Godot 4.7.2-stable, commit
`ed1daf0bf001b61586d9930840f2f1394092c079`.
The official macOS universal release archive was verified against its published
SHA-512 list:
`38aa16e5bba2083941fc5b3e54be0089bd4cc35e32415f5b9fd9a8a6a7b9818255d44532ea8ef94b5aef56c4b407c2d634fa4f657e4ebe681ebbf59b7bac69ca`.

Set `GODOT` to that executable and run `tools/goldens/generate.sh`.
`dump.gd` constructs all 23 named scenarios without imported assets. It advances
Skeleton3D's native manual modifier pipeline and captures `modification_processed`
before the skeleton restores its input pose. An inactive zero-delta advance
registers the modifier before recording, letting deferred setup finish. The run
records 300 frames at fixed 60 fps; `--quit-after 301` allows the setup frame.
Every actual delta is retained, including its full JSON precision.

Fixtures are JSON compressed with deterministic gzip (`mtime=0`). Compression
changes no numeric values. The 23 fixtures total 2,398,643 bytes. A second complete
run produced identical SHA-256 hashes; `tests/goldens/manifest.json` records them.
`math.gd` emits shortest-arc quaternion references, including opposite vectors.

`upstream/` contains verbatim pinned source and its hash manifest. It is excluded
from the published crate, but retained in Git for independent inspection. The
MIT notice and human credits are retained in the distribution.
